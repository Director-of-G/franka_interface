#include "franka-interface/feedback_controller/my_cartesian_velocity_feedback_controller.h"
#include <franka/rate_limiting.h>
#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

void MyCartesianVelocityFeedbackController::parse_parameters(){
    //pass
}

void MyCartesianVelocityFeedbackController::initialize_controller(FrankaRobot *robot) {
  model_ = robot->getModel();
}

void MyCartesianVelocityFeedbackController::get_next_step(const franka::RobotState &robot_state,
                                                     TrajectoryGenerator *traj_generator) {

  // Read current coriolis and jacobian terms from model.
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);

  // convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  JointTrajectoryGenerator* joint_trajectory_generator = dynamic_cast<JointTrajectoryGenerator*>(traj_generator);

  if(joint_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  std::array<double, 7> desired_joints = joint_trajectory_generator->get_desired_joints();

  // Compute torque command from joint impedance control law.
  // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
  // time step delay.
  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; i++) {
    tau_d_calculated[i] = k_gains_[i] * (desired_joints[i] - robot_state.q[i])
          - d_gains_[i] * robot_state.dq[i] + coriolis[i];
  }

  // The following line is only necessary if rate limiting is not activate. If we activated
  // rate limiting for the control loop (activated by default), the torque would anyway be
  // adjusted!
  std::array<double, 7> tau_d_rate_limited =
      franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

  tau_d_array_ = tau_d_rate_limited;
}