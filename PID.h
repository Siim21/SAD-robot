#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

struct PIDcontroller{

  float proportional;
  float integral;
  float derivative;

  float dLowPassFilter;

  float limMin;
  float limMax;

  float time;

  float integrator;
  float prevError;
  float differentialator;
  float prevMeasurement;

  float out;

};

void PIDController_Init(PIDcontroller *pid);
float PIDController_Update(PIDcontroller *pid, float error);

#endif