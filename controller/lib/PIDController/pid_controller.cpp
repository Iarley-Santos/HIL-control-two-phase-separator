#include "pid_controller.h"

pid_controller::pid_controller(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0.0f;
    _previous_error = 0.0f;
    _u = 0.0f;
}
void pid_controller::set_kp(float kp) {
    _kp = kp;
}

void pid_controller::set_ki(float ki) {
    _ki = ki;
}

void pid_controller::set_kd(float kd) {
    _kd = kd;
}

void pid_controller::set_parameters(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float pid_controller::pid_calculation(float error, float delta_time)
{
    _integral += error * delta_time;
    float derivative = (error - _previous_error) / delta_time;
    
    _u = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    _previous_error = error;
    
    return _u;
}