#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

struct PIDcontroller{

  float Kp;
  float Ki;
  float Kd;

  float dLowPassFilter;

  float limMin;
  float limMax;

  float T;

  float integrator;
  float prevError;
  float differentialator;
  float prevMeasurement;

  float out;

};

void PIDController_Init(PIDcontroller *pid);
float PIDController_Update(PIDcontroller *pid, float error);

#endif