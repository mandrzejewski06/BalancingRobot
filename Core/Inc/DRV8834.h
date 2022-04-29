/*
 * DRV8834.h
 *
 *  Created on: 15 lut 2022
 *      Author: mand2
 */

#ifndef INC_DRV8834_H_
#define INC_DRV8834_H_
//TIMER CONFIGURATION
#define USING_WHICH_TIMERS APB1
#define PCLK_PRESCALER 2
// STEPPER MOTOR CONFIGURATION CONSTANTS
#define DRV8834_MOTOR_STEP_PER_REVOLUTION	200
#define DRV8834_MOT_DEFAULT_MAX_FREQ		350
#define DRV8834_MOT_MIN_FREQ				1
#define DRV8834_MOT_DEFAULT_MAX_SPEED		50

typedef enum {
	STOPPED = 0, FORWARD, BACKWARD, LEFT, RIGHT, BALANCING
} RoborState_t;

typedef enum {__MOTOR_FWD = 0, __MOTOR_BACK = 1} StepMotorDirection_t;

typedef struct
{
	uint16_t DIR_PIN;
	GPIO_TypeDef* DIR_PORT;
	uint16_t M0_PIN;
	GPIO_TypeDef* M0_PORT;
	uint16_t M1_PIN;
	GPIO_TypeDef* M1_PORT;
} StepMotorPins_t;

typedef struct
{
	StepMotorDirection_t direction;
	StepMotorPins_t step_motor_pins;

	TIM_HandleTypeDef *PWM_timer;
	uint32_t PWM_timerChannel;

	uint8_t microstep;
	uint32_t last_counter;
} StepMotor_t;

void DRV8834_setRobotState(uint16_t st);
void DRV8834_setMaxSpeed(uint16_t spd);
void DRV8834_setMaxFreq(uint16_t freq);
uint16_t DRV8834_getRobotState(void);
uint16_t DRV8834_getMaxSpeed(void);
uint16_t DRV8834_getMaxFreq(void);

void DRV8834_Init(StepMotor_t *stepMotor, TIM_HandleTypeDef *htim, uint32_t channel);
void DRV8834_InitPins(StepMotor_t *stepMotor, uint16_t dirPin, GPIO_TypeDef* dirPort, uint16_t m0Pin, GPIO_TypeDef* m0Port, uint16_t m1Pin, GPIO_TypeDef* m1Port);
void DeinitializePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t Mode);
uint8_t DRV8834_SetMicrostep(StepMotor_t *stepMotor, uint8_t microstep);
void DRV8834_StopMotor(StepMotor_t *stepMotor);
void DRV8834_SetSpeed(StepMotor_t *stepMotor, int32_t speed);
void DRV8834_StartMotor(StepMotor_t *stepMotor, int32_t speed);

#endif /* INC_DRV8834_H_ */
