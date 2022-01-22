/*
 * PID.c
 *
 *  Created on: Jan 18, 2022
 *      Author: mand2
 */
#include "PID.h"

void PID_Init(PID_t *pid, double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    pid->myOutput = Output;
    pid->myInput = Input;
    pid->mySetpoint = Setpoint;
    pid->inAuto = false;
    pid->SampleTime = 50;

    PID_SetOutputLimits(pid, 0, 255);				//default output limit
    PID_SetControllerDirection(pid, ControllerDirection);
    PID_SetTunings(pid, Kp, Ki, Kd, POn);
}

void PID_SetTunings(PID_t *pid, double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pid->pOn = POn;
   pid->pOnE = (POn == P_ON_ERROR);

   pid->dispKp = Kp;
   pid->dispKi = Ki;
   pid->dispKd = Kd;

   double SampleTimeInSec = ((double)pid->SampleTime)/1000;
   pid->kp = Kp;
   pid->ki = Ki * SampleTimeInSec;
   pid->kd = Kd / SampleTimeInSec;

  if(pid->controllerDirection == REVERSE)
   {
	  pid->kp = (0 - pid->kp);
	  pid->ki = (0 - pid->ki);
	  pid->kd = (0 - pid->kd);
   }
}

void PID_SetSampleTime(PID_t *pid, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)pid->SampleTime;
      pid->ki *= ratio;
      pid->kd /= ratio;
      pid->SampleTime = (unsigned long)NewSampleTime;
   }
}

void PID_SetOutputLimits(PID_t *pid, double Min, double Max)
{
   if(Min >= Max) return;
   pid->outMin = Min;
   pid->outMax = Max;

   if(pid->inAuto)
   {
	   if(*(pid->myOutput) > pid->outMax) *(pid->myOutput) = pid->outMax;
	   else if(*(pid->myOutput) < pid->outMin) *(pid->myOutput) = pid->outMin;

	   if(pid->outputSum > pid->outMax) pid->outputSum= pid->outMax;
	   else if(pid->outputSum < pid->outMin) pid->outputSum= pid->outMin;
   }
}

void PID_SetMode(PID_t *pid, int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !pid->inAuto)
    {  /*we just went from manual to auto*/
    	pid->outputSum = *pid->myOutput;
    	pid->lastInput = *pid->myInput;
	   if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
	   else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
    }
    pid->inAuto = newAuto;
}

void PID_SetControllerDirection(PID_t *pid, int Direction)
{
   if(pid->inAuto && (Direction != pid->controllerDirection))
   {
	   pid->kp = (0 - pid->kp);
	   pid->ki = (0 - pid->ki);
	   pid->kd = (0 - pid->kd);
   }
   pid->controllerDirection = Direction;
}

bool PID_Compute(PID_t *pid)
{
   if(!pid->inAuto)
   {
	   return false;
   }

      /*Compute all the working error variables*/
      double input = *pid->myInput;
      double error = *pid->mySetpoint - input;
      double dInput = (input - pid->lastInput);
      pid->outputSum+= (pid->ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pid->pOnE)
	  {
    	  pid->outputSum-= pid->kp * dInput;
	  }

      if(pid->outputSum > pid->outMax) pid->outputSum= pid->outMax;
      else if(pid->outputSum < pid->outMin) pid->outputSum= pid->outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	  double output;
      if(pid->pOnE) output = pid->kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
      output += pid->outputSum - pid->kd * dInput;

      if(output > pid->outMax) output = pid->outMax;
      else if(output < pid->outMin) output = pid->outMin;
      *pid->myOutput = output;

      /*Remember some variables for next time*/
      pid->lastInput = input;
      return true;
}

