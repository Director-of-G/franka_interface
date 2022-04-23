#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"
#include "franka-interface/trajectory_generator/my_joint_velocity_generator.h"

void MyJointVelocityGenerator::parse_parameters() {
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

void MyJointVelocityGenerator::parse_sensor_data(const franka::RobotState &robot_state) {
  SensorDataManagerReadStatus sensor_msg_status = sensor_data_manager_->readTrajectoryGeneratorSensorMessage(joint_vels_sensor_msg_);
  if (sensor_msg_status == SensorDataManagerReadStatus::SUCCESS) {
    for (int i = 0; i < 7; i++) {
      desired_joint_vels_[i] = std::min(std::max(joint_vels_sensor_msg_.joint_vels(i), -0.2), 0.2);
    }
  }
}

void MyJointVelocityGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                     SkillType skill_type) {
  initialize_initial_and_desired_joints(robot_state, skill_type);
}

void MyJointVelocityGenerator::initialize_initial_and_desired_joints(const franka::RobotState &robot_state,
                                                                     SkillType skill_type) {
  switch(skill_type) {
    case SkillType::JointPositionSkill:
      initial_joints_ = robot_state.q_d;
      desired_joint_vels_ = robot_state.q_d;
      break;
    case SkillType::ImpedanceControlSkill:
      initial_joints_ = robot_state.q;
      desired_joint_vels_ = robot_state.q;
      break;
    case SkillType::MyJointVelocitySkill://================================
      initial_joints_ = robot_state.dq;
      desired_joint_vels_ = robot_state.dq;
      break;
    default:
      initial_joints_ = robot_state.q_d;
      desired_joint_vels_ = robot_state.q_d;
  }
}
void MyJointVelocityGenerator::setGoalJoints(const std::array<double, 7> joints) {
  for (int i = 0; i < 7; i++) {
    goal_joints_[i] = last_goal_joints_[i] + std::max(std::min(static_cast<double>(goal_joints_[i]) - last_goal_joints_[i], dqi_limit), -dqi_limit);
    last_goal_joints_[i] = goal_joints_[i];
  }
}

void MyJointVelocityGenerator::setInitialJoints(const std::array<double, 7> joints) {
  for (int i = 0; i < 7; i++) {
    initial_joints_[i] = static_cast<double>(joints[i]);
  }
}

const std::array<double, 7>& MyJointVelocityGenerator::get_desired_joints() const {
  return desired_joint_vels_;
}

const std::array<double, 7>& MyJointVelocityGenerator::get_goal_joints() const {
  return goal_joints_;
}

void MyJointVelocityGenerator::get_next_step(const franka::RobotState &robot_state){}