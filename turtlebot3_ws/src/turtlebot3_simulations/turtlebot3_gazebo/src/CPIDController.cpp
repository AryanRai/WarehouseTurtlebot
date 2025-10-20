/*
 * File: CPIDController.cpp
 * Description: Implements the CPIDController class for PID control 
   logic in TurtleBot3 Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This file provides a proportional-integral-derivative controller 
   used for closed-loop control of robot behaviors.
 */

#include "turtlebot3_gazebo/CWallFollowingController.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Sensor.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Drive.hpp"
#include "turtlebot3_gazebo/CPIDController.hpp"


// CPIDController constructor
CPIDController::CPIDController(double aProportional_gain, 
                             double aIntegral_gain, 
                             double aDerivative_gain, 
                             double aMaximum_output)
  : kp_(aProportional_gain),
    ki_(aIntegral_gain),
    kd_(aDerivative_gain),
    max_output_(aMaximum_output),
    prev_error_(0.0),
    integral_(0.0)
{
  // Constructor initializes all member variables
}

// CPIDController destructor
CPIDController::~CPIDController()
{
  // Destructor
}

// Compute PID output
double CPIDController::compute(double aError, double aDelta_time) 
{
  // Validate input
  if (aDelta_time <= 0.0) 
  {
    return 0.0;
  }

  // Compute integral with clamping
  integral_ += aError * aDelta_time;
  const double max_integral = max_output_ / ki_;
  const double min_integral = -max_integral;
  
  if (integral_ < min_integral) 
  { 
    integral_ = min_integral;
  }
  else if (integral_ > max_integral) 
  { 
    integral_ = max_integral;
  }

  // Compute derivative
  const double derivative = (aError - prev_error_) / aDelta_time;
  prev_error_ = aError;

  // Compute PID output
  double output = kp_ * aError + ki_ * integral_ + kd_ * derivative;

  // Clamp output to limits
  if (output < -max_output_) 
  { 
    output = -max_output_;
  }
  else if (output > max_output_) 
  { 
    output = max_output_;
  }
  
  return output;
}

// Reset PID controller state
void CPIDController::reset() 
{
  prev_error_ = 0.0;
  integral_ = 0.0;
}