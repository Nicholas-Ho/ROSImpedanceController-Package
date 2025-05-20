// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_example_controllers/TargetPose.h>
#include <franka_example_controllers/SafetyRepulsiveFields.h>

namespace franka_example_controllers {

class RepulsiveFieldInfo {
  private:
    Eigen::Vector3d centre_target;
    double strength_target;
    double active_radius_target;

    // For filtering
    Eigen::Vector3d centre;
    double strength;
    double active_radius;

    std::mutex mutex;
  public:
    void setParameters(double x, double y, double z, double s, double r);
    void updateInternals(double filter_param);
    Eigen::VectorXd calculateCartesianForces(Eigen::Vector3d position);
};

class CartesianImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  CartesianImpedanceExampleController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> cartesian_integral_;
  Eigen::Matrix<double, 6, 6> cartesian_integral_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  const float damping_ratio{1};
  const float integral_ratio{3};
  Eigen::Matrix<double, 6, 1> error_integral;
  bool track_error_integral = false;

  // Safety limits for Tanh cartesian spring
  const double max_cartesian_spring_force{10};

  // For safety repulsive fields
  RepulsiveFieldInfo safety_fields[3];

//   // Dynamic reconfigure
//   std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
//       dynamic_server_compliance_param_;
//   ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
//   void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
//                                uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const franka_example_controllers::TargetPose::ConstPtr& msg);

  // Safety repuslive fields subscriber
  ros::Subscriber sub_safety_fields;
  void safetyFieldsCallback(const franka_example_controllers::SafetyRepulsiveFields::ConstPtr& msg);

  // Panda robot joint limits (from specifications) converted to radians
  Eigen::Matrix<double, 7, 1> lower_joint_limits = (Eigen::Matrix<double, 7, 1>() << -2.897247, -1.762783, -2.897247, -3.071780, -2.897247, -0.0174532, -2.897247).finished();
  Eigen::Matrix<double, 7, 1> upper_joint_limits = (Eigen::Matrix<double, 7, 1>() << 2.897246, 1.762782, 2.897246, -0.069813, 2.897246, 3.752457, 2.897246).finished();

  // Force functions
  Eigen::VectorXd tanhCartesianSpring(Eigen::Matrix<double, 6, 6> stiffness, Eigen::Matrix<double, 6, 1> error, double max_force);

  Eigen::VectorXd jointLimitForceFunction(
    Eigen::Matrix<double, 7, 1> q,
    Eigen::Matrix<double, 7, 1> lower_limits,
    Eigen::Matrix<double, 7, 1> upper_limits,
    double k,
    double buffer);

  //----------------------added-------------------//
  ros::Publisher pub_current_pose_;
  ros::Publisher pub_joint_positions;
  //----------------------added-------------------//
};

}  // namespace franka_example_controllers
