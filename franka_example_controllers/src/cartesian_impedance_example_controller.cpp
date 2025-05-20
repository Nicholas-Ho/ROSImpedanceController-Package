// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_example_controllers/JointPositions.h>

namespace franka_example_controllers {

bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/target_data", 20, &CartesianImpedanceExampleController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_safety_fields = node_handle.subscribe(
      "/safety_fields", 20, &CartesianImpedanceExampleController::safetyFieldsCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  // add a publisher 
  pub_current_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/robot_current_pose",50);
  pub_joint_positions = node_handle.advertise<franka_example_controllers::JointPositions>("joint_positions",100);
  //-------------------modified------------------------------//


  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // dynamic_reconfigure_compliance_param_node_ =
  //     ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  // dynamic_server_compliance_param_ = std::make_unique<
  //     dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

  //     dynamic_reconfigure_compliance_param_node_);
  // dynamic_server_compliance_param_->setCallback(
  //     boost::bind(&CartesianImpedanceExampleController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  error_integral.setZero();

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  cartesian_integral_.setZero();

  // Safety repulsive fields
  safety_fields[0].setParameters(0, 0, 0, 0, 0);
  safety_fields[1].setParameters(0, 0, 0, 0, 0);
  safety_fields[2].setParameters(0, 0, 0, 0, 0);
  safety_fields[0].updateInternals(1);
  safety_fields[1].updateInternals(1);
  safety_fields[2].updateInternals(1);

  return true;
}

void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceExampleController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute error integral
  if (track_error_integral) error_integral += error * period.toSec();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-tanhCartesianSpring(cartesian_stiffness_, error, max_cartesian_spring_force) -
                    cartesian_damping_ * (jacobian * dq) -
                    cartesian_integral_ * error_integral) +
              jointLimitForceFunction(q, lower_joint_limits, upper_joint_limits, 10, 0.4) +  // Joint limit protections
              jacobian.transpose() * safety_fields[0].calculateCartesianForces(position) +  // Safety repulsive fields
              jacobian.transpose() * safety_fields[1].calculateCartesianForces(position) +
              jacobian.transpose() * safety_fields[2].calculateCartesianForces(position);

  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    cartesian_integral_ =
        filter_params_ * cartesian_integral_target_ + (1.0 - filter_params_) * cartesian_integral_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }

  // Do the same for the safety fields
  safety_fields[0].updateInternals(filter_params_);
  safety_fields[1].updateInternals(filter_params_);
  safety_fields[2].updateInternals(filter_params_);

  //----------------------added---------------------------------//
  // publish robot pose

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "panda_link0"; // need to check 

  pose_msg.pose.position.x = position[0];
  pose_msg.pose.position.y = position[1];
  pose_msg.pose.position.z = position[2];
  pose_msg.pose.orientation.x = orientation.x();
  pose_msg.pose.orientation.y = orientation.y();
  pose_msg.pose.orientation.z = orientation.z();
  pose_msg.pose.orientation.w = orientation.w();
  pub_current_pose_.publish(pose_msg);

  franka_example_controllers::JointPositions joint_data_msg;
  std::array<double, 7> q_data = robot_state.q;
  for (int i=0; i<7; i++) {
    joint_data_msg.positions[i] = q_data[i];
  }
  pub_joint_positions.publish(joint_data_msg);
  //----------------------added---------------------------------//
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

Eigen::VectorXd CartesianImpedanceExampleController::tanhCartesianSpring(Eigen::Matrix<double, 6, 6> stiffness, Eigen::Matrix<double, 6, 1> error, double max_force) {
  // Expecting stiffness to be a diagonal matrix with diagonal { cartesian_stiffness (3 times), rotational_stiffness (3 times)}
  double cartesian_stiffness = stiffness(0, 0);
  double rotational_stiffness = stiffness(5, 5);
  double k = cartesian_stiffness / max_force;

  Eigen::Vector3d cartesian_error;
  cartesian_error << error[0], error[1], error[2]; 
  Eigen::Vector3d cartesian_force = cartesian_error.normalized() * max_force * std::tanh(k * cartesian_error.norm());

  Eigen::VectorXd force(6);
  force << cartesian_force,
           rotational_stiffness * error[3],
           rotational_stiffness * error[4],
           rotational_stiffness * error[5];
  return force;
}

Eigen::VectorXd CartesianImpedanceExampleController::jointLimitForceFunction(
    Eigen::Matrix<double, 7, 1> q,
    Eigen::Matrix<double, 7, 1> lower_limits,
    Eigen::Matrix<double, 7, 1> upper_limits,
    double k,
    double buffer) {

  // Zero between offset, linear with gradient otherwise
  static auto force_func = [] (double x, double gradient, double lower_offset, double upper_offset) {
    if (x < lower_offset) return gradient * (x - lower_offset);
    if (x > upper_offset) return gradient * (x - upper_offset);
    return 0.0;
  };
  
  Eigen::VectorXd force(7);
  for (int i=0; i<7; i++) {
    force[i] = force_func(q[i], -k, lower_limits[i]+buffer, upper_limits[i]-buffer);
  }

  return force;
}

void RepulsiveFieldInfo::setParameters(double x, double y, double z, double s, double r) {
  std::lock_guard<std::mutex> safety_field_mutex_lock(mutex);
  centre_target << x, y, z;
  strength_target = s;
  active_radius_target = r;
}

void RepulsiveFieldInfo::updateInternals(double filter_param) {
  std::lock_guard<std::mutex> safety_field_mutex_lock(mutex);
  centre = filter_param * centre_target + (1.0 - filter_param) * centre;
  strength = filter_param * strength_target + (1.0 - filter_param) * strength;
  active_radius = filter_param * active_radius_target + (1.0 - filter_param) * active_radius;
}

Eigen::VectorXd RepulsiveFieldInfo::calculateCartesianForces(Eigen::Vector3d position) {
  // Zero beyond active radius, inversely proportional to cubed distance otherwise
  static auto force_func = [] (double dist, double k, double radius) {
    dist = std::abs(dist);  // Ensure absolute
    if (dist < radius) return k / std::pow(dist, 3);
    return 0.0;
  };
  
  Eigen::Vector3d diff = position - centre;
  Eigen::Vector3d cartesian_force = diff.normalized() * force_func(diff.norm(), strength, active_radius);

  Eigen::VectorXd force_vec(6);
  // { cartesian_forces(3), rotation_forces(3) }
  force_vec << cartesian_force, 0, 0, 0;

  return force_vec;
}

void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const franka_example_controllers::TargetPose::ConstPtr& msg) {
  // Change target
  {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
  }

  // Change parameters
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << msg->cartesian_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << msg->rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(msg->cartesian_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(msg->rotational_stiffness) * Eigen::Matrix3d::Identity();
  // Integral ratio = 0.01
  const float integral_ratio = 1;
  cartesian_integral_target_.topLeftCorner(3, 3)
      << integral_ratio * 2.0 * sqrt(msg->cartesian_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_integral_target_.bottomRightCorner(3, 3)
      << integral_ratio * 2.0 * sqrt(msg->rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = 0.5;

  // Reset error integral if zero
  const float thresh = 0.001;
  if (msg->cartesian_stiffness <= thresh) {
    error_integral.setZero();
    track_error_integral = false;
  } else {
    track_error_integral = true;
  }
}

void CartesianImpedanceExampleController::safetyFieldsCallback(
    const franka_example_controllers::SafetyRepulsiveFields::ConstPtr& msg) {
  safety_fields[0].setParameters(
    msg->body.centre.x,
    msg->body.centre.y,
    msg->body.centre.z,
    msg->body.strength,
    msg->body.active_radius
  );

  safety_fields[1].setParameters(
    msg->left_hand.centre.x,
    msg->left_hand.centre.y,
    msg->left_hand.centre.z,
    msg->left_hand.strength,
    msg->left_hand.active_radius
  );

  safety_fields[2].setParameters(
    msg->right_hand.centre.x,
    msg->right_hand.centre.y,
    msg->right_hand.centre.z,
    msg->right_hand.strength,
    msg->right_hand.active_radius
  );
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)
