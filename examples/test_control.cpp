/*!
 * @file test_control.cpp
 * @author Shane Rozen-Levy <srozen01@seas.upenn.edu>, J. Diego Caporale <jdcap@seas.upenn.edu>, 
 *         Wei-Hsi Chen <weicc@seas.upenn.edu>, Lucien Peach <peach@seas.upenn.edu>
 * @brief Example that allows for simple sinusoidal motion on a test motor.  
 * @date 8/22/2022
 * 
 * @copyright Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
 *            BSD 3-Clause License
 * 
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include <sys/mman.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>

class Test_Control : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog> {
  // using MjbotsControlLoop::MjbotsControlLoop;
  public:
  float start_position = 0;
  float start_time = 0;
  bool init_flag = false;
    Test_Control(
      std::vector<kodlab::mjbots::JointMoteus> joints, 
      const kodlab::mjbots::ControlLoopOptions &options) : 
      kodlab::mjbots::MjbotsControlLoop<ManyMotorLog>(joints,options)
    {
    }
  private:
    void Update() override {
      if (!init_flag){
        start_position = robot_->GetJointPositions()[0];
        start_time = time_now_/1000.0; //converts time_now_ to ms units
        init_flag = true;
      }
      // Define parameters
      float period = 1000.0; //ms
      float amp = 1.0; //rad
      float Kp = 0.5;
      float Kd = 0.005;

      std::vector<float> positions = robot_->GetJointPositions();
      std::vector<float> velocities = robot_->GetJointVelocities();
      float time = time_now_/1000.0 - start_time; //converts time_now_ to ms units

      // float theta_desired = start_position ;
      float theta_desired = amp*sin((6.28/period)*(time))+ start_position;
      float theta_dot_desired = amp*(6.28/period)*cos((6.28/period)*(time));
      float desired_torque = Kp*(theta_desired - positions[0]) + Kd*(theta_dot_desired - velocities[0]);
      // float desired_torque = 1;
      std::cout<<start_position <<"\t"<<positions[0]<<std::endl;
      std::vector<float> torques(num_joints_, desired_torque);
      robot_->SetTorques(torques);
    }

    void PrepareLog() override {
      for (int servo = 0; servo < num_joints_; servo++) {
        log_data_->positions[servo] = robot_->GetJointPositions()[servo];
        log_data_->velocities[servo] = robot_->GetJointVelocities()[servo];
        log_data_->modes[servo] = static_cast<int>(mjbots_interface_->GetJointModes()[servo]);
        log_data_->torques[servo] = robot_->GetJointTorqueCmd()[servo];
      }
      for (int servo = num_joints_; servo < 13; servo++) {
        log_data_->positions[servo] = 0;
        log_data_->velocities[servo] = 0;
        log_data_->modes[servo] = 0;
        log_data_->torques[servo] = 0;
      }
    }
};

int main(int argc, char **argv) {

  //Setup joints
  std::vector<kodlab::mjbots::JointMoteus> joints;
  joints.emplace_back(85, 1, 1, 0, 1, 1);

  // Define robot options
  kodlab::mjbots::ControlLoopOptions options;
  options.log_channel_name = "motor_data";
  options.frequency = 1000;
  options.realtime_params.main_cpu = 3;
  options.realtime_params.can_cpu  = 2;

  // Create control loop
  Test_Control control_loop(joints, options);
  // Starts the loop, and then join it
  control_loop.Start();
  control_loop.Join();
  return 0;
}
