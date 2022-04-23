#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MY_JOINT_VELOCITY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MY_JOINT_VELOCITY_GENERATOR_H_

#include <array>

#include "franka-interface/trajectory_generator/trajectory_generator.h"

class MyJointVelocityGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state, SkillType skill_type=SkillType::JointPositionSkill) override;

  /**
   * Initialize initial and desired joints from robot state
   */
  void initialize_initial_and_desired_joints(const franka::RobotState &robot_state, SkillType skill_type);

  /**
   * Set the goal joints to new value. This is called when new data is received from sensor buffer.
   * @param joints
   */
  void setGoalJoints(const std::array<double, 7> joints);

  /**
   * Set the initial joints to new value. This is called when new data is received from sensor buffer.
   * @param joints
   */
  void setInitialJoints(const std::array<double, 7> joints);

  /**
   * Returns the desired joints. This method is called at every time step of the control loop to
   * find the joint positions to move to at the next step.
   */
  const std::array<double, 7>& get_desired_joints() const;

  /**
   * Returns the goal joints. These are the final set of joint positions that the skill needs to reach.
   */
  const std::array<double, 7>& get_goal_joints() const;

 protected:
  JointTrajectoryGeneratorMessage joint_trajectory_params_;
  JointPositionVelocitySensorMessage joint_vels_sensor_msg_;
  void parse_sensor_data(const franka::RobotState &robot_state) override;
  void get_next_step(const franka::RobotState &robot_state) override;

  std::array<double, 7> initial_joints_{};
  std::array<double, 7> desired_joint_vels_{};
  std::array<double, 7> goal_joints_{};
  std::array<double, 7> last_goal_joints_{};
  const double dqi_limit = 0.15;
};

#endif	// FRANKA_INTERFACE_TRAJECTORY_GENERATOR_MY_JOINT_VELOCITY_GENERATOR_H_