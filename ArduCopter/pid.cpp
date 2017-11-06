#include "pid.h"

void PID::init(PID_PARAMETERS param){
    pid_param.e = param.e;
    pid_param.e_ = param.e_;
    pid_param.e__ = param.e__;
    pid_param.Kd = param.Kd;
    pid_param.Ki = param.Ki;
    pid_param.Kp = param.Kp;
    pid_param.PID_Saturation = param.PID_Saturation;
    pid_param.Ts = param.Ts;
    pid_param.u = param.u;
    pid_param.u_ = param.u_;
}

void PID::pid_set_k_params(float Kp,float Ki, float Kd)
{
    pid_param.Kp = Kp;
    pid_param.Ki = Ki;
    pid_param.Kd = Kd;
}

float PID::pid_process(float error)
{
    // pid_param.e__ = pid_param.e_;
    pid_param.e_ = pid_param.e;
    pid_param.e = error;
    // pid_param.u_ = pid_param.u;
    pid_param.u = pid_param.Kp * (pid_param.e)
            // + pid_param.Ki * pid_param.Ts * pid_param.e
            + (pid_param.Kd / pid_param.Ts) * (pid_param.e - pid_param.e_);

    // if (pid_param.u > pid_param.PID_Saturation)
    // {
    //     pid_param.u = pid_param.PID_Saturation;
    // }
    // else if (pid_param.u < (-pid_param.PID_Saturation))
    // {
    //     pid_param.u = -pid_param.PID_Saturation;
    // }

    return pid_param.u;
}

void PID::pid_reset()
{
    pid_param.e = 0;
    pid_param.e_ = 0;
    pid_param.e__ = 0;
    pid_param.u = 0;
    pid_param.u_ = 0;
}
