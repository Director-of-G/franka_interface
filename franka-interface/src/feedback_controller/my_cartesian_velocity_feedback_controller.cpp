//modified from cartesian_impedance_feedback_controller.cpp

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
  // convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());


  JointTrajectoryGenerator* joint_trajectory_generator = dynamic_cast<JointTrajectoryGenerator*>(traj_generator);

  if(joint_trajectory_generator == nullptr) {
    throw std::bad_cast();
  }

  std::array<double, 7> desired_joints_veloc = joint_trajectory_generator->get_desired_joints();

  desired_joints_veloc = 
  veloc_d_array_ = dq + (desired_joints_veloc - dq)
}