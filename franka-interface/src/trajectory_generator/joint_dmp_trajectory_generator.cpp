//
// Created by mohit on 12/3/18.
//

#include "franka-interface/trajectory_generator/joint_dmp_trajectory_generator.h"

#include <cmath>

/*
    Origin code implementation in Franka Interface
    Modified version of IRM lab is attached after this
    Modification 2022/4/14
*/

/*
void JointDmpTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = joint_dmp_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = joint_dmp_trajectory_params_.run_time();
    tau_ = joint_dmp_trajectory_params_.tau();
    alpha_ = joint_dmp_trajectory_params_.alpha();
    beta_ = joint_dmp_trajectory_params_.beta();
    num_basis_ = joint_dmp_trajectory_params_.num_basis();
    num_sensor_values_ = joint_dmp_trajectory_params_.num_sensor_values();

    for (int i = 0; i < num_basis_; i++) {
      basis_mean_[i] = joint_dmp_trajectory_params_.basis_mean(i);
      basis_std_[i] = joint_dmp_trajectory_params_.basis_std(i);
    }

    for (int i = 0; i < num_dims_; i++) {
      for (int j = 0; j < num_sensor_values_; j++) {
        for (int k = 0; k < num_basis_; k++) {
          int index = (i * num_sensor_values_* num_basis_) + (j * num_basis_) + k;
          weights_[i][j][k] = joint_dmp_trajectory_params_.weights(index);
        }
      }
    }

    for (int i = 0; i < num_sensor_values_; i++) {
      initial_sensor_values_[i] = joint_dmp_trajectory_params_.initial_sensor_values(i);
    }
  } else {
    std::cout << "Parsing JointDMPTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void JointDmpTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                        SkillType skill_type) {
  initialize_initial_and_desired_joints(robot_state, skill_type);
  
  y0_ = robot_state.q_d;
  y_ = robot_state.q_d;
  dy_ = robot_state.dq_d;
  x_ = 1.0;
}


void JointDmpTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  static int i, j, k;
  static double ddy, t;

  static std::array<double, 20> factor{};
  static double den = 0.;
  static double net_sensor_force;
  static double sensor_feature;

  // Calculate feature values.
  den = 0.;
  for (k = 0; k < num_basis_; k++) {
    factor[k] = exp(-basis_std_[k] * pow((x_ - basis_mean_[k]), 2));
    // first basis is for the min-jerk feature
    if (k > 0) {
      den += factor[k];
    }
  }

  for (k = 1; k < num_basis_; k++) {
    // factor[k] = (factor[k] * x_) / (den * basis_mean_[k]);
    factor[k] = (factor[k] * x_) / (den + 1e-8);
  }
  t = fmin(-log(x_) * 2.0, 1.0);
  // TODO(Mohit): Shouldn't the below index be 0?
  factor[0] = pow(t, 3) * (6*pow(t, 2) - 15 * t + 10);

  for (i = 0; i < num_dims_; i++) {
    ddy = (alpha_ * (beta_ * (y0_[i] - y_[i]) - dy_[i] / tau_));
    net_sensor_force = 0;
    for (j = 0; j < num_sensor_values_; j++) {
      sensor_feature = 0;
      for (k=0; k < num_basis_; k++) {
        sensor_feature += (factor[k] * weights_[i][j][k]);
      }
      net_sensor_force += (initial_sensor_values_[j] * sensor_feature);
    }
    ddy += (alpha_ * beta_ * net_sensor_force);
    ddy *= (tau_ * tau_);
    dy_[i] += (ddy * dt_);
    y_[i] += (dy_[i] * dt_);
  }

  // Update canonical system.
  x_ -= (x_ * tau_) * dt_;

  // Finally set the joints we want.
  desired_joints_ = y_;
}

void JointDmpTrajectoryGenerator::getInitialMeanAndStd() {
  std::array<double, 10> basis_mean{};
  std::array<double, 10> basis_std{};
  for (int i = 0; i < num_basis_; i++)  {
    basis_mean[i] = exp(-(i)*0.5/(num_basis_-1));
  }
  for (int i = 0; i < num_basis_ - 1; i++) {
    basis_std[i] = 0.5 / (0.65 * pow(basis_mean[i+1] - basis_mean[i], 2));
  }
  basis_std[num_basis_ - 1] = basis_std[num_basis_ - 2];
}
*/


/*
    IRM lab's modified version of Franka Interface
    Modification 2022/4/14
*/


void JointDmpTrajectoryGenerator::parse_parameters() {
  // First parameter is reserved for the type

  int data_size = (params_[1] + (params_[2] << 8) + (params_[3] << 16) + (params_[4] << 24));

  bool parsed_params = joint_dmp_trajectory_params_.ParseFromArray(params_ + 5, data_size);

  if(parsed_params){
    run_time_ = joint_dmp_trajectory_params_.run_time();
    tau_ = joint_dmp_trajectory_params_.tau();  // tau
    alpha_y_ = joint_dmp_trajectory_params_.alpha();  // alpha_y
    beta_y_ = joint_dmp_trajectory_params_.beta();  // beta_y
    num_basis_ = joint_dmp_trajectory_params_.num_basis();  // num_basis
    alpha_x_ = joint_dmp_trajectory_params_.num_sensor_values();  // alpha_x | Use the fuzzy parameter num_sensor_values to represent alpha_x, this is not a typo!

    std::cout << '\n\n\n' << std::endl;
    std::cout << "tau: " << tau_ << std::endl;
    std::cout << "alpha_y: " << alpha_y_ << std::endl;
    std::cout << "beta_y: " << beta_y_ << std::endl;
    std::cout << "num_basis: " << num_basis_ << std::endl;
    std::cout << "alpha_x: " << alpha_x_ << std::endl;
    std::cout << "[Joint DMP]Parsing JointDMPTrajectoryGenerator params failed. Data size = " << data_size << std::endl;

    for (int i = 0; i < num_basis_; i++) {
      basis_mean_[i] = joint_dmp_trajectory_params_.basis_mean(i);  // miu
      basis_std_[i] = joint_dmp_trajectory_params_.basis_std(i);  // h
      std::cout << basis_mean_[i] << " , " << basis_std_[i] << std:: endl;
    }

    for (int i = 0; i < num_dims_; i++) {
        for (int j = 0; j < num_basis_; j++) {
          int index = (i * num_basis_) + j;
          my_weights_[i][j] = joint_dmp_trajectory_params_.weights(index);
        }
    }

    for (int i = 0; i < num_dims_; i++) {
      my_goal_[i] = joint_dmp_trajectory_params_.initial_sensor_values(i);
    }
    joints_memory_file.open("/home/roboticslab/yxj/frankapy/data/0415/joints_memory.txt");
  } else {
    std::cout << "[Joint DMP]Parsing JointDMPTrajectoryGenerator params failed. Data size = " << data_size << std::endl;
  }
}

void JointDmpTrajectoryGenerator::initialize_trajectory(const franka::RobotState &robot_state,
                                                        SkillType skill_type) {
  initialize_initial_and_desired_joints(robot_state, skill_type);
  
  y0_ = robot_state.q_d;
  y_ = robot_state.q_d;
  dy_ = robot_state.dq_d;
  ddy_ = robot_state.ddq_d;
  x_ = 1.0;
  run_cnt = 0;
}

void JointDmpTrajectoryGenerator::get_next_step(const franka::RobotState &robot_state) {
  static int i, j, k;
  std::array<double, 7> ddy_={};
  static double dx;
  static std::array<double, 42> psi_current{};

  // calculate basis function values at current timestep
  // dx = -tau_ * alpha_x_ * x_ * 0.00246;   // CS needs dt=0.01
  dx = -tau_ * alpha_x_ * x_ * 0.01;
  x_ = x_ + dx;

  double psi_current_sum = 0;
  for (i = 0; i < num_basis_; i++){
    psi_current[i] = exp(-basis_std_[i] * pow((x_ - basis_mean_[i]), 2));
    psi_current_sum += psi_current[i];
  }

  // calculate step ddy, dy and y
  for (i = 0; i < num_dims_; i++){
    k = alpha_y_;  // currently use the same alpha_y_ for all DoF
    double f_rep = 0;
    double psi_current_weighted_sum = 0;
    for (j = 0; j < num_basis_; j++){
      psi_current_weighted_sum += (psi_current[j] * my_weights_[i][j]);
    }
    f_rep = (psi_current_weighted_sum / psi_current_sum - (my_goal_[i] - y0_[i])) * x_ * k;
    ddy_[i] = k * (beta_y_ * (my_goal_[i] - y_[i]) - dy_[i]) + f_rep;
    dy_[i] += tau_ * ddy_[i] * 0.01;
    y_[i] += tau_ * dy_[i] * 0.01;
  }

  run_cnt += 1;
  std::cout << run_cnt << std::endl;
  std::cout << y_[0] << " " << y_[1] << " " << y_[2] << " " << y_[3] << " " << y_[4] << " " << y_[5] << " " << y_[6] << std::endl;
  joints_memory_file << y_[0] << " " << y_[1] << " " << y_[2] << " " << y_[3] << " " << y_[4] << " " << y_[5] << " " << y_[6] << std::endl;
  // std::array<double, 7> curent_joint_command = {y_[0], y_[1], y_[2], y_[3], y_[4], y_[5], y_[6]};
  // joints_memory.push_back(curent_joint_command);
  // Finally set the joints we want.
  desired_joints_ = y_;
}

void JointDmpTrajectoryGenerator::getInitialMeanAndStd() {
  std::array<double, 42> basis_mean{};
  std::array<double, 42> basis_std{};
  for (int i = 0; i < num_basis_; i++)  {
    basis_mean[i] = exp(-(i)*0.5/(num_basis_-1));
  }
  for (int i = 0; i < num_basis_ - 1; i++) {
    basis_std[i] = 0.5 / (0.65 * pow(basis_mean[i+1] - basis_mean[i], 2));
  }
  basis_std[num_basis_ - 1] = basis_std[num_basis_ - 2];
}

