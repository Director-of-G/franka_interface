#ifndef FRANKA_INTERFACE_TRAJECTORY_GENERATOR_JOINT_DMP_TRAJECTORY_GENERATOR_H_
#define FRANKA_INTERFACE_TRAJECTORY_GENERATOR_JOINT_DMP_TRAJECTORY_GENERATOR_H_

#include "franka-interface/trajectory_generator/joint_trajectory_generator.h"
#include <fstream>
#include <iostream>

class JointDmpTrajectoryGenerator : public JointTrajectoryGenerator {
 public:
  using JointTrajectoryGenerator::JointTrajectoryGenerator;

  void parse_parameters() override;

  void initialize_trajectory(const franka::RobotState &robot_state,
                             SkillType skill_type=SkillType::JointPositionSkill) override;

  void get_next_step(const franka::RobotState &robot_state) override;

  std::array<double, 7> y_={};
  std::array<double, 7> dy_={};
  std::array<double, 7> ddy_={};

 private:
  JointDMPTrajectoryGeneratorMessage joint_dmp_trajectory_params_;

  // Variables initialized from shared memory should be doubles.
  double alpha_=5.0;
  double beta_=5.0/4.0;
  double tau_=0.0;
  double x_=1.0;
  int num_basis_=42;
  int num_dims_=7;
  int num_sensor_values_=10;

  /*
    Modified by IRM lab
    Modification: 2022/04/14
  */
  double alpha_y_ = 60.0;
  double beta_y_ = 60.0/4.0;
  double alpha_x_ = 1.0;

  std::array<double, 42> basis_mean_{};
  std::array<double, 42> basis_std_{};

  std::array<std::array<std::array<double, 42>, 10>, 7> weights_{};
  std::array<std::array<double, 42>, 7> my_weights_{};

  std::array<double, 10> initial_sensor_values_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::array<double, 7> my_goal_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::ofstream joints_memory_file;
  
  // std::vector<std::array<double, 7>> joints_memory;
  int run_cnt =0;

  std::array<double, 7> y0_={};

  void getInitialMeanAndStd();
};

#endif  // FRANKA_INTERFACE_TRAJECTORY_GENERATOR_JOINT_DMP_TRAJECTORY_GENERATOR_H_