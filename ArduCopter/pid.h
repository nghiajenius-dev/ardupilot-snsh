#ifndef PID_H
#define PID_H


class PID
{   
public:
    typedef struct
    {
        float Kp;
        float Ki;
        float Kd;
        float Ts;
        float PID_Saturation;
        float e;
        float e_;
        float e__;
        float u;
        float u_;
    } PID_PARAMETERS;
    
    void init(PID_PARAMETERS param);
    float pid_process(float error);
    void pid_reset();
    void pid_set_k_params(float Kp,float Ki, float Kd);

private:
   PID_PARAMETERS pid_param;
};

#endif // PID_H
