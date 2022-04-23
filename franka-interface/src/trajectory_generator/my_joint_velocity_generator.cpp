#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"

void JointVelocityGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));
  bool parsed_params = joint_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = joint_trajectory_params_.run_time();
    for(int i = 0; i < 7; i++) {
      goal_joints_[i] = joint_trajectory_params_.joints(i);
    }
  } else {
    std::cout << "Parsing JointVelocityGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void JointVelocityGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                     SkillType skill_type) {
  initialize_initial_and_desired_joints(robot_state, skill_type);
}

void JointVelocityGenerator::initialize_initial_and_desired_joints(const franka::RobotState &robot_state,
                                                                     SkillType skill_type) {
  switch(skill_type) {
    case SkillType::JointPositionSkill:
      initial_joints_ = robot_state.q_d;
      desired_joints_ = robot_state.q_d;
      break;
    case SkillType::ImpedanceControlSkill:
      initial_joints_ = robot_state.q;
      desired_joints_ = robot_state.q;
      break;
    default:
      initial_joints_ = robot_state.q_d;
      desired_joints_ = robot_state.q_d;
  }
}
void JointVelocityGenerator::setGoalJoints(const std::array<double, 7> joints) {
  for (int i = 0; i < 7; i++) {
    goal_joints_[i] = last_goal_joints_[i]+std::max(std::min(static_cast<double>(goal_joints_[i])-last_goal_joints_[i], dqi_limit), -dqi_limit));
    last_goal_joints_[i] = goal_joints_[i]
  }
}

void JointVelocityGenerator::setInitialJoints(const std::array<double, 7> joints) {
  for (int i = 0; i < 7; i++) {
    initial_joints_[i] = static_cast<double>(joints[i]);
  }
}

const std::array<double, 7>& JointVelocityGenerator::get_desired_joints() const {
  return desired_joints_;
}

const std::array<double, 7>& JointVelocityGenerator::get_goal_joints() const {
  return goal_joints_;
}