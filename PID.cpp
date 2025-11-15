#include "PID.h"

void PIDController_Init(PIDcontroller *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentialator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDcontroller *pid, float error) {

	float proportional;

	proportional = error * pid->proportional;

  
	pid->integrator += 0.5f * (error + pid->prevError) * pid->time * pid->integral;

	float limMaxInt, limMinInt;
	
	if (pid->limMax > proportional){

		limMaxInt = pid->limMax - proportional;

	} 
  else {

		limMaxInt = 0.0f;

	}

	if (pid->limMin < proportional){

		limMinInt = pid->limMin - proportional;

	} 

  else {

		limMinInt = 0.0f;

	}
	/* Anti-wind-up via integrator clamping */
  if (pid->integrator > limMaxInt) {

      pid->integrator = limMaxInt;

  } 
	
	else if (pid->integrator < limMinInt) {

      pid->integrator = limMinInt;
	}
	
		
  pid->differentialator = -(pid->derivative * (error - pid->prevError)	+ (pid->dLowPassFilter - pid->time) * pid->differentialator) / (pid->dLowPassFilter + pid->time);


  pid->out = proportional + pid->integrator + pid->differentialator;


  if (pid->out > pid->limMax) {

    pid->out = pid->limMax;

  } 
		
	else if (pid->out < pid->limMin) {

    pid->out = pid->limMin;

  }

  pid->prevError = error;
	
  return pid->out;

	
}