/*
 * PID.h
 *
 *  Created on: Jan 18, 2022
 *      Author: mand2
 */
#include "main.h"

#ifndef INC_PID_H_
#define INC_PID_H_

#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_MEASUREMENT 0
#define P_ON_ERROR 1

#define ERROR 0					// value for error status
#define SAMPLE_TIME_DEFAULT 100	// in miliseconds

typedef struct PID {
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered
	double dispKi;				//   format for display purposes
	double dispKd;				//

	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	uint8_t controllerDirection;
	uint8_t pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

	double outputSum, lastInput;

	uint32_t SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;
} PID_t;

void PID_Init(PID_t *pid, double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection);
void PID_SetTunings(PID_t *pid, double Kp, double Ki, double Kd, int POn);
void PID_SetSampleTime(PID_t *pid, int NewSampleTime);
void PID_SetOutputLimits(PID_t *pid, double Min, double Max);
void PID_SetMode(PID_t *pid, int Mode);
void PID_SetControllerDirection(PID_t *pid, int Direction);
bool PID_Compute(PID_t *pid);

#endif /* INC_PID_H_ */
