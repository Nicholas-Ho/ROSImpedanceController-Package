// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Adapted from Omaar's object handover vmc controller

#include <franka_example_controllers/vmc_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_example_controllers/JointPositions.h>

namespace franka_example_controllers {

bool VMCController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  // std::vector<double> cartesian_stiffness_vector;
  // std::vector<double> cartesian_damping_vector;

  // Receive joint torques from the rospy-Julia server and apply them to the robot.

  joints_subscriber = node_handle.subscribe(
      "/joint_commands", 20, &VMCController::TorquesFromJulia, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // -----NICHOLAS_PROJECT-----
  // Publish current end-effector position and joint positions
  pub_current_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("robot_current_pose",50);
  pub_joint_positions = node_handle.advertise<franka_example_controllers::JointPositions>("joint_positions",100);
  // -----NICHOLAS_PROJECT-----

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("VMCController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "VMCController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "VMCController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "VMCController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "VMCController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "VMCController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "VMCController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "VMCController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // dynamic_reconfigure_compliance_param_node_ =
  //     ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  // dynamic_server_compliance_param_ = std::make_unique<
  //     dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

  //     dynamic_reconfigure_compliance_param_node_);
  // dynamic_server_compliance_param_->setCallback(
  //     boost::bind(&VMCController::complianceParamCallback, this, _1, _2));

  torques_from_julia.setZero();
  // orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  // position_d_target_.setZero();
  // orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // position_d_o_.setZero();
  // orientation_d_o_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  // position_d_o_target_.setZero();
  // orientation_d_o_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // cartesian_stiffness_.setZero();
  // cartesian_damping_.setZero();

  return true;
}

void VMCController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  // std::array<double, 42> jacobian_array =
  //     model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // // convert to eigen
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  // Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // // set equilibrium point to current state
  // position_d_ = initial_transform.translation();
  // orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  // position_d_target_ = initial_transform.translation();
  // orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // position_d_o_ = initial_transform.translation();
  // orientation_d_o_ = Eigen::Quaterniond(initial_transform.rotation());
  // position_d_o_target_ = initial_transform.translation();
  // orientation_d_o_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // // set nullspace equilibrium configuration to initial q
  // q_d_nullspace_ = q_initial;

  // counter = 1;
  torques_from_julia.setZero();
  counter = 1;
}

void VMCController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  // std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  // std::array<double, 42> jacobian_array =
  //     model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // // convert to Eigen
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  // Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  // Eigen::Vector3d position(transform.translation());
  // Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  // Eigen::Matrix<double, 6, 1> error;
  // error.head(3) << position - position_d_;

  // Eigen::Matrix<double, 6, 1> error_o;
  // error_o.head(3) << position - position_d_o_;

  // Eigen::Matrix<double, 6, 1> intercept;
  // intercept << 6.5, 6.5, 6.5, 0.0, 0.0, 0.0;

  // Eigen::Vector3d mag_o = error_o.head(3);

  // // orientation error
  // if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
  //   orientation.coeffs() << -orientation.coeffs();
  // }

  // // "difference" quaternion
  // Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  // // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // error.tail(3) << 0.0, 0.0, 0.0;
  // // Transform to base frame
  // error.tail(3) << -transform.rotation() * error.tail(3);

  // Eigen::Quaterniond error_o_quaternion(orientation.inverse() * orientation_d_o_);
  // // error_o.tail(3) << error_o_quaternion.x(), error_o_quaternion.y(), error_o_quaternion.z();
  // error_o.tail(3) << 0.0, 0.0, 0.0;
  // // Transform to base frame
  // error_o.tail(3) << -transform.rotation() * error_o.tail(3);

  // compute control
  // allocate variables
  // Eigen::VectorXd tau_task(7), tau_task_o(7), tau_nullspace(7), tau_d(7);
  // Eigen::VectorXd tau_task(7);
     Eigen::VectorXd tau_d(7);
  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  // Eigen::MatrixXd jacobian_transpose_pinv;
  // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Eigen::MatrixXd eliminate_orientation = Eigen::MatrixXd::Zero(6,6);
  // eliminate_orientation.topLeftCorner(3,3).setIdentity();
  
  // Cartesian PD control with damping ratio = 1
  // tau_task << jacobian.transpose() *
  //                 (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
                  
  // nullspace PD control with damping ratio = 1
  // tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
  //                   jacobian.transpose() * jacobian_transpose_pinv) *
  //                      (nullspace_stiffness_ * (q_d_nullspace_ - q) -
  //                       (2.0 * sqrt(nullspace_stiffness_)) * dq);

  // if ((mag_o.norm() < 0.25) && (mag_o.norm() > 0.05)){
  //   // stiffness_scale = -32*mag_o.norm() + 7.4;
  //   // force = -20*mag_o.norm() + 6.5;
  //   stiffness_scale = -20;
  //   tau_task_o << -jacobian.transpose() *
  //                 (-(stiffness_scale * eliminate_orientation * error_o + intercept) - 2 * -sqrt(-stiffness_scale) * eliminate_orientation * (jacobian * dq) );
  // }
  // else if ((mag_o.norm() <= 0.25) && (mag_o.norm() > 0.1)){
  //   tau_task_o << -25*(jacobian.transpose() *
  //                 (-cartesian_stiffness_ * error_o - cartesian_damping_ * (jacobian * dq)));
  // }

  // else if ((mag_o.norm() <= 0.1) && (mag_o.norm() > 0.075)) {
  //   tau_task_o << -100*(jacobian.transpose() *
  //                 (-cartesian_stiffness_ * error_o - cartesian_damping_ * (jacobian * dq)));
  // }
  // else if (mag_o.norm() <= 0.075){
  //   tau_task_o.setZero();
  // }
  // else {
  //   tau_task_o.setZero();
  // }
  // // tau_task_o.setZero();
  // tau_task.setZero();

  // Desired torque
  tau_d << torques_from_julia;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  // ROS_INFO_STREAM(tau_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // if ((counter % 50 == 0) && (tau_task_o[1] > 0.1)){
  //   ROS_INFO_STREAM("Obstacle error value: " << error_o.transpose());
  //   // ROS_INFO_STREAM("Obstacle error magnitude: " << mag_o.norm());
  //   // ROS_INFO_STREAM("Goal error value: " << error.transpose());
  //   ROS_INFO_STREAM("Obstacle torque values: " << tau_task_o.transpose());
  //   // ROS_INFO_STREAM("Goal torque values: " << tau_task.transpose());
  //   // ROS_INFO_STREAM("Stiffness scale: " << stiffness_scale);
  // }
  // counter += 1;

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  // cartesian_stiffness_ =
  //     filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  // cartesian_damping_ =
  //     filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  // nullspace_stiffness_ =
  //     filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  // std::lock_guard<std::mutex> position_d_target_mutex_lock(
  //     position_and_orientation_d_target_mutex_);
  // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  // orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  // std::lock_guard<std::mutex> position_d_o_target_mutex_lock(
  //     position_and_orientation_d_o_target_mutex_);
  // position_d_o_ = filter_params_ * position_d_o_target_ + (1.0 - filter_params_) * position_d_o_;
  // orientation_d_o_ = orientation_d_o_.slerp(filter_params_, orientation_d_o_target_);

  // -----NICHOLAS_PROJECT-----
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
  // -----NICHOLAS_PROJECT-----
}

Eigen::Matrix<double, 7, 1> VMCController::saturateTorqueRate(
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

// void VMCController::complianceParamCallback(
//     franka_example_controllers::compliance_paramConfig& config,
//     uint32_t /*level*/) {
//   cartesian_stiffness_target_.setIdentity();
//   cartesian_stiffness_target_.topLeftCorner(3, 3)
//       << config.translational_stiffness * Eigen::Matrix3d::Identity();
//   cartesian_stiffness_target_.bottomRightCorner(3, 3)
//       << config.rotational_stiffness * Eigen::Matrix3d::Identity();
//   cartesian_damping_target_.setIdentity();
//   // Damping ratio = 1
//   cartesian_damping_target_.topLeftCorner(3, 3)
//       << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
//   cartesian_damping_target_.bottomRightCorner(3, 3)
//       << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
//   nullspace_stiffness_target_ = config.nullspace_stiffness;
// }

void VMCController::TorquesFromJulia(
    const std_msgs::Float64MultiArrayConstPtr& msg) {
  torques_from_julia << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6];
  if (counter % 30 == 0) {
  ROS_INFO_STREAM(torques_from_julia);
  }
  counter = counter + 1;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::VMCController,
                       controller_interface::ControllerBase)
