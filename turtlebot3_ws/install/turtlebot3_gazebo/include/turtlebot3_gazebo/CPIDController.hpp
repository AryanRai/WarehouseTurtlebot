/*
 * File: CPIDController.hpp
 * Description: Declares the CPIDController class for PID control logic in TurtleBot3 Gazebo simulation.
 * 
 * Date: 2025-10-10
 *
 * This header defines the interface for a proportional-integral-derivative controller
 * used for closed-loop control of robot behaviors.
 */

#ifndef TURTLEBOT3_GAZEBO__CPID_CONTROLLER_HPP_
#define TURTLEBOT3_GAZEBO__CPID_CONTROLLER_HPP_

/**
 * @class CPIDController
 * @brief Implements a proportional-integral-derivative controller
 * 
 * Computes control output based on error signal with anti-windup protection.
 * Thread-safe for single-threaded use.
 */
class CPIDController 
{
  public:
    /**
     * @brief Constructor for PID controller
     * @param aProportional_gain Proportional gain (Kp)
     * @param aIntegral_gain Integral gain (Ki)
     * @param aDerivative_gain Derivative gain (Kd)
     * @param aMaximum_output Maximum absolute value of output
     */
    CPIDController(double aProportional_gain, 
                  double aIntegral_gain, 
                  double aDerivative_gain, 
                  double aMaximum_output);
    
    /**
     * @brief Destructor
     */
    ~CPIDController();

    /**
     * @brief Compute PID control output
     * @param aError Current error signal (setpoint - measurement)
     * @param aDelta_time Time step since last computation (seconds)
     * @return Control output, clamped to [-max_output, max_output]
     */
    double compute(double aError, double aDelta_time);
    
    /**
     * @brief Reset internal state (integral, previous error)
     */
    void reset();

  private:
    // PID gains
    double kp_;
    double ki_;
    double kd_;
    
    // Output limits
    double max_output_;
    
    // Internal state
    double prev_error_;
    double integral_;
};

#endif  // TURTLEBOT3_GAZEBO__CPID_CONTROLLER_HPP_