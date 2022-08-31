// BSD 3-Clause License
// Copyright (c) 2021 The Trustees of the University of Pennsylvania. All Rights Reserved
// Authors:
// Shane Rozen-Levy <srozen01@seas.upenn.edu>, J. Diego Caporale <jdcap@seas.upenn.edu>

/* Basic example script demonstrating how to use the mjbots_control_loop to 2 joints. The functions to implement are
 * CalcTorques and PrepareLog. In this example we send a torque cmd of all zeros and log the motor information.
 */

#include "kodlab_mjbots_sdk/mjbots_control_loop.h"
#include "kodlab_mjbots_sdk/joint_moteus.h"
#include "ManyMotorLog.hpp"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"
#include <sys/mman.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>
#include <math.h>

class Test_Control : public kodlab::mjbots::MjbotsControlLoop<ManyMotorLog> {
  // using MjbotsControlLoop::MjbotsControlLoop;
  public:
    float start_position = 0;
    float start_time = 0;
    float desiredControl = 0;
    bool init_flag = false;
    float temp = 0;

    enum TaskMode {
      StartPause = 0, SpringPhase, ThrustPhase, TimeReset, DummyMode
    };

    TaskMode mode = StartPause; //Current state
    TaskMode lastmode = StartPause; //Previous state

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

      // Parameters specifically for use within state-machine representation
      float scale = 1;

      float T = 30; // Period
      float desiredSpringP = 0.0; //rad
      float guardSpringP = -0.5; //rad
      float errSpringP = 0;  //rad
      float errSpringV = 0;  //rad/s
      float gainSpringP = scale * 0.3;  //Nm/rad
      float gainSpringV = scale * 0.003; //Nms/rad
      float thrustTorque = scale * 0.8; //Nm
      float desiredThrustP = 0;
      float errThrustP = 0;  //rad
      float errThrustV = 0;  //rad/s
      float gainThrustP = 10 * gainSpringP;
      float gainThrustV = 10 * gainSpringV;
      float CompressTime = 10;  //ms

      float lastV = 0;
      int countContact = 0;

      float tLast; //System time at last state change
      float tNow; //System time at current point
      float TT;   

      // Definitions
      std::vector<float> positions = robot_ ->GetJointPositions();
      std::vector<float> velocities = robot_ ->GetJointVelocities();
      tNow = time_now_/1000.0 - start_time; //converts time_now_ to ms units

      // Commonly used variables for simplification
      errSpringP = desiredSpringP - positions[0];
      errSpringV = - velocities[0];
      errThrustP = desiredThrustP - positions[0];
      errThrustV = - velocities[0];

      // Update loop for state-machine representation
      if (mode == StartPause){  // Soft start
        temp = (tNow < 3000 ? tNow / 3000 : 1);
        desiredControl = (gainSpringP * errSpringP + gainSpringV * errSpringV) * temp;
        mode = (tNow>3000? SpringPhase : StartPause);
        tLast = tNow;
      } else if (mode == SpringPhase){  // Spring system, in flight
        TT = (tNow-tLast);  //ms
        desiredControl = (gainSpringP * errSpringP + gainSpringV * errSpringV);
        mode = (errSpringP < guardSpringP? ThrustPhase : SpringPhase);
      } else if (mode == ThrustPhase){  // Following angular displacement criteria
        desiredControl = -thrustTorque;
        // desiredControl = (gainThrustP * errThrustP + gainThrustV * errThrustV);
        // mode = (fabs(errThrustP) < 0.1? SpringPhase : ThrustPhase);
        mode = (positions[0] < 0.01 ? SpringPhase : ThrustPhase);
        lastmode = ThrustPhase;
      } else {
        desiredControl = 0;
      }

      // Update conditions & printing to pi terminal
      std::cout << errSpringP << "\t" << positions[0] << "\t" << desiredControl << "\t" << mode << std::endl;
      std::vector<float> torques(num_joints_, desiredControl);
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
  joints.emplace_back(85, 1, 1, -0.121894, 1, 2); // {id, x, x, zero position, x, x}-0.121894
  // joints.emplace_back(86, 1, 1, 0.456159, 1, 9);


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
