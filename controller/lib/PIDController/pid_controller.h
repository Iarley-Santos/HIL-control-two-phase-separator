/********************************************************************
 * File: pid_controller.h 
 * Description: Simple and efficient C++ PID control library 
 *              for robots with two motors (such as differential 
 *              or rear-wheel drive). Ideal for use with Arduino, 
 *              ESP32, and other embedded systems.
 * 
 * 
 * Created by: Iarley Santos - June 04, 2025
********************************************************************/

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cstdint>

class pid_controller
{
public:
    /**
     * @brief Constructor: Initializes the controller with PID constants.
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    pid_controller(float kp, float ki, float kd);

    /**
     * @brief Calculates the PID control signal.
     * 
     * Computes the control output based on the current setpoint, feedback, 
     * and time interval since the last calculation.
     * 
     * @param error        Error function
     * @param delta_time   Time interval since last update (in seconds)
     * 
     * @return PID control output (integer, may be positive or negative)
     */
    float pid_calculation(float error, float delta_time);

    // --- Setters for individual PID constants ---

    /**
     * @brief Sets a new proportional gain (Kp).
     * 
     * @param kp New Kp value
     */
    void set_kp(float kp);

    /**
     * @brief Sets a new integral gain (Ki).
     * 
     * @param ki New Ki value
     */
    void set_ki(float ki);

    /**
     * @brief Sets a new derivative gain (Kd).
     * 
     * @param kd New Kd value
     */
    void set_kd(float kd);

    /**
     * @brief Sets all PID gains at once.
     * 
     * @param kp New proportional gain
     * @param ki New integral gain
     * @param kd New derivative gain
     */
    void set_parameters(float kp, float ki, float kd);

private:
    /* PID Constants */
    float _kp;
    float _ki;
    float _kd;

    /* Internal state */
    float _integral;         // Accumulated integral error
    float _previous_error;   // Previous error for derivative calculation
    float _u;                  // Last computed control signal
};

#endif // PIDCONTROLLER_H