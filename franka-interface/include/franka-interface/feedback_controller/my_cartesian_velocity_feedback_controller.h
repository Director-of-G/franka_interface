#ifndef FRANKA_INTERFACE_FEEDBACK_CONTROLLER_MY_CARTESIAN_VELOCITY_FEEDBACK_CONTROLLER_H_
#define FRANKA_INTERFACE_FEEDBACK_CONTROLLER_MY_CARTESIAN_VELOCITY_FEEDBACK_CONTROLLER_H_

#include "franka-interface/feedback_controller/feedback_controller.h"

#include <array>

class MyCartesianVelocityFeedbackController : public FeedbackController {
 public:
  using FeedbackController::FeedbackController;

  void parse_parameters() override;

  void initialize_controller(FrankaRobot *robot) override;

  void get_next_step(const franka::RobotState &robot_state, TrajectoryGenerator *traj_generator) override;

 private:
  MyCartesianVelocityFeedbackControllerMessage joint_impedance_feedback_params_;

  const franka::Model *model_;
  // p
  std::array<double, 7> k_gains_ = {{600.0, 600.0, 600.0, 600.0, 
                                     250.0, 150.0, 50.0}};
  // d
  std::array<double, 7> d_gains_ = {{50.0, 50.0, 50.0, 50.0, 
                                     30.0, 25.0, 15.0}};
};

#endif  // FRANKA_INTERFACE_FEEDBACK_CONTROLLER_MY_CARTESIAN_VELOCITY_FEEDBACK_CONTROLLER_H_