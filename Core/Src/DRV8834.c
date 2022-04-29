/*
 * DRV8834.c
 *
 *  Created on: 15 lut 2022
 *      Author: mand2
 */

#include "main.h"
#include "DRV8834.h"

#define ABS(value) (value < 0 ? -value : value)

static uint16_t maxSpeed = DRV8834_MOT_DEFAULT_MAX_SPEED;
static uint16_t maxFrequency = DRV8834_MOT_DEFAULT_MAX_FREQ;
static RoborState_t state;



void DRV8834_setRobotState(uint16_t st) { state = st;}
void DRV8834_setMaxSpeed(uint16_t spd) { maxSpeed = spd;}
void DRV8834_setMaxFreq(uint16_t freq) { maxFrequency = freq; }
uint16_t DRV8834_getRobotState(void) { return state; }
uint16_t DRV8834_getMaxSpeed(void) { return maxSpeed; }
uint16_t DRV8834_getMaxFreq(void) { return maxFrequency; }

void DRV8834_Init(StepMotor_t *stepMotor, TIM_HandleTypeDef *htim, uint32_t channel)
{
	stepMotor->PWM_timer = htim;
	stepMotor->PWM_timerChannel = channel;
	stepMotor->direction = __MOTOR_FWD;
	stepMotor->last_counter = 0;
}

void DRV8834_InitPins(StepMotor_t *stepMotor, uint16_t dirPin, GPIO_TypeDef* dirPort, uint16_t m0Pin, GPIO_TypeDef* m0Port, uint16_t m1Pin, GPIO_TypeDef* m1Port)
{
	stepMotor->step_motor_pins.DIR_PIN = dirPin;
	stepMotor->step_motor_pins.DIR_PORT = dirPort;

	stepMotor->step_motor_pins.M0_PIN = m0Pin;
	stepMotor->step_motor_pins.M0_PORT = m0Port;

	stepMotor->step_motor_pins.M1_PIN = m1Pin;
	stepMotor->step_motor_pins.M1_PORT = m1Port;
}

void DeinitializePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t Mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(Mode == GPIO_MODE_OUTPUT_PP)
	{
		//HAL_GPIO_DeInit(GPIOx, GPIO_Pin); // CZY POTRZEBNE?
		GPIO_InitStruct.Pin = GPIO_Pin;
		GPIO_InitStruct.Mode = Mode;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}
	else if (Mode == GPIO_MODE_INPUT)
	{
		GPIO_InitStruct.Pin = GPIO_Pin;
		GPIO_InitStruct.Mode = Mode;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	}
}

uint8_t DRV8834_SetMicrostep(StepMotor_t *stepMotor, uint8_t microstep)
{
    /*
     * Step mode truth table
     * M1 M0    step mode
     *  0  0     1
     *  0  1     2
     *  0  Z     4
     *  1  0     8
     *  1  1    16
     *  1  Z    32
     *
     *  Z = high impedance mode (M0 is three-state)
     */

	if(state != STOPPED)
	{
		return HAL_ERROR;
	}

	switch(microstep)
	{
	case 1:
	case 8:
		DeinitializePin(stepMotor->step_motor_pins.M0_PORT, stepMotor->step_motor_pins.M0_PIN, GPIO_MODE_OUTPUT_PP);
		HAL_GPIO_WritePin(stepMotor->step_motor_pins.M0_PORT, stepMotor->step_motor_pins.M0_PIN, GPIO_PIN_RESET);
		break;
	case 2:
	case 16:
		DeinitializePin(stepMotor->step_motor_pins.M0_PORT, stepMotor->step_motor_pins.M0_PIN, GPIO_MODE_OUTPUT_PP);
		HAL_GPIO_WritePin(stepMotor->step_motor_pins.M0_PORT, stepMotor->step_motor_pins.M0_PIN, GPIO_PIN_SET);
		break;
	case 4:
	case 32:
		DeinitializePin(stepMotor->step_motor_pins.M0_PORT, stepMotor->step_motor_pins.M0_PIN, GPIO_MODE_INPUT);
		break;
	default:
		return HAL_ERROR;
	}

	if(microstep < 8)
	{
		HAL_GPIO_WritePin(stepMotor->step_motor_pins.M1_PORT, stepMotor->step_motor_pins.M1_PIN, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(stepMotor->step_motor_pins.M1_PORT, stepMotor->step_motor_pins.M1_PIN, GPIO_PIN_SET);
	}

	stepMotor->microstep = microstep;

	return HAL_OK;
}

void DRV8834_StopMotor(StepMotor_t *stepMotor)
{
	//__HAL_TIM_SET_COMPARE(stepMotor->PWM_timer, stepMotor->PWM_timerChannel, 0); // CZY POTRZEBNE?
	HAL_TIM_PWM_Stop(stepMotor->PWM_timer, stepMotor->PWM_timerChannel);
	state = STOPPED;
}

static void DRV8834_SetDirection(StepMotor_t *stepMotor, StepMotorDirection_t dir)
{
	if((dir == __MOTOR_FWD) || (dir == __MOTOR_BACK))
	{
		HAL_GPIO_WritePin(stepMotor->step_motor_pins.DIR_PORT, stepMotor->step_motor_pins.DIR_PIN, dir);
		stepMotor->direction = dir;
	}
}

void DRV8834_SetSpeed(StepMotor_t *stepMotor, int32_t speed)
{
	uint32_t counter, freq;

	if((speed < 0) && (stepMotor->direction == __MOTOR_FWD))
	{
		DRV8834_SetDirection(stepMotor, __MOTOR_BACK);
	}
	else if((speed > 0) && (stepMotor->direction == __MOTOR_BACK))
	{
		DRV8834_SetDirection(stepMotor, __MOTOR_FWD);
	}

	if(speed < 0)
	{
		speed = ABS(speed);
	}
	if(speed > maxSpeed)
	{
		speed = maxSpeed;
	}

	freq = (speed * (stepMotor->microstep*(maxFrequency - DRV8834_MOT_MIN_FREQ))) / maxSpeed;

#if USING_WHICH_TIMERS == APB1
	if(freq != 0)
		counter = PCLK_PRESCALER*HAL_RCC_GetPCLK1Freq() / (stepMotor->PWM_timer->Init.Prescaler * freq);
	else
		counter = 0;
#elif USING_WHICH_TIMERS == APB2
		counter = PCLK_PRESCALER*HAL_RCC_GetPCLK2Freq() / (stepMotor->PWM_timer->Init.Prescaler * freq);
#else
#error "Choose between APB1 timers or APB2 timers or reimplement code if using both"
#endif
	if(stepMotor->last_counter != counter)
	{
		if(counter == 0)
		{
			__HAL_TIM_SET_COUNTER(stepMotor->PWM_timer, 0);
			__HAL_TIM_SET_AUTORELOAD(stepMotor->PWM_timer, 0);
			__HAL_TIM_SET_COMPARE(stepMotor->PWM_timer, stepMotor->PWM_timerChannel, 0);
		}
		else
		{
			__HAL_TIM_SET_COUNTER(stepMotor->PWM_timer, 0);
			__HAL_TIM_SET_AUTORELOAD(stepMotor->PWM_timer, counter - 1);
			__HAL_TIM_SET_COMPARE(stepMotor->PWM_timer, stepMotor->PWM_timerChannel, (counter/2) - 1);
		}
		stepMotor->last_counter = counter;
	}
}

void DRV8834_StartMotor(StepMotor_t *stepMotor, int32_t speed)
{
	state = BALANCING;

	DRV8834_SetSpeed(stepMotor, speed);

	HAL_TIM_PWM_Start(stepMotor->PWM_timer, stepMotor->PWM_timerChannel);
}
