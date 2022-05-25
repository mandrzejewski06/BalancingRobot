/*
 * DRV8834.c
 *
 *  Created on: 15 lut 2022
 *      Author: mand2
 */

#include "main.h"
#include "DRV8834.h"

// Makro do obliczania wartości bezwzględnej
#define ABS(value) (value < 0 ? -value : value)
// Zmienne wewnętrzne biblioteki
static uint16_t maxSpeed = DRV8834_MOT_DEFAULT_MAX_SPEED;		// Maksymalna prędkość
static uint16_t maxFrequency = DRV8834_MOT_DEFAULT_MAX_FREQ;	// Maksymalna częstotliwość
static RoborState_t state;	// Stan robota
// Funkcje zapisu i odczytu zmiennych powyżej
void DRV8834_setRobotState(uint16_t st) { state = st;}
void DRV8834_setMaxSpeed(uint16_t spd) { maxSpeed = spd;}
void DRV8834_setMaxFreq(uint16_t freq) { maxFrequency = freq; }
uint16_t DRV8834_getRobotState(void) { return state; }
uint16_t DRV8834_getMaxSpeed(void) { return maxSpeed; }
uint16_t DRV8834_getMaxFreq(void) { return maxFrequency; }
// Funkcja inicjalizująca - przyjmuje wskaźnik na obiekt silnika, uchwyt (wskaźnik) do timera i numer kanału timera
void DRV8834_Init(StepMotor_t *stepMotor, TIM_HandleTypeDef *htim, uint32_t channel)
{
	stepMotor->PWM_timer = htim;
	stepMotor->PWM_timerChannel = channel;
	stepMotor->direction = __MOTOR_FWD;		// Domyślny kierunek prosto
	stepMotor->last_counter = 0;			// Ostatnia wartość zliczania licznika to 0
}
// Funkcja inicjalizująca piny, przyjmuje numer pinu i portu M0, M1 (mikrokrok) i DIR (kierunku)
void DRV8834_InitPins(StepMotor_t *stepMotor, uint16_t dirPin, GPIO_TypeDef* dirPort, uint16_t m0Pin, GPIO_TypeDef* m0Port, uint16_t m1Pin, GPIO_TypeDef* m1Port)
{
	stepMotor->step_motor_pins.DIR_PIN = dirPin;
	stepMotor->step_motor_pins.DIR_PORT = dirPort;

	stepMotor->step_motor_pins.M0_PIN = m0Pin;
	stepMotor->step_motor_pins.M0_PORT = m0Port;

	stepMotor->step_motor_pins.M1_PIN = m1Pin;
	stepMotor->step_motor_pins.M1_PORT = m1Port;
}
// Funkcja do deinicjalizacji pinu z wejścia w wyjście i na odwrót
// Przyjmuje numer portu, pinu oraz tryb jaki ma być ustawiony:
// GPIO_MODE_OUTPUT_PP lub GPIO_MODE_INPUT
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
// Funkcja zmiany mikrokroku, przyjmuje wartość mikrokroku jaki ma być ustawiony
// silniki podczas zmiany muszą być zatrzymane, dozwolone wartości to
// 1, 2, 4, 8, 16, 32. Zwraca HAL_ERROR jeśli warunki nie zostały spełnione, jeśli wszystko ok zwraca HAL_OK
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
	// Jeśli silniki nie są w stanie STOPPED to zwróć HAL_ERROR
	if(state != STOPPED)
	{
		return HAL_ERROR;
	}
	// Ustawienie pinów zgodnie z zadaną wartością i wg tabeli powyżej
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
	// Przypisanie nowego mikrokroku do obiektu silnika
	stepMotor->microstep = microstep;

	return HAL_OK;
}
// Wprowadzenie silnika w stan STOPPED
void DRV8834_StopMotor(StepMotor_t *stepMotor)
{
	// TODO: wprowadzenie robota w stan STOPPED przed zatrzymaniem drugiego silnika, jak to rozwiązać
	//__HAL_TIM_SET_COMPARE(stepMotor->PWM_timer, stepMotor->PWM_timerChannel, 0); // CZY POTRZEBNE?
	HAL_TIM_PWM_Stop(stepMotor->PWM_timer, stepMotor->PWM_timerChannel); // Zastopuj timer
	state = STOPPED;	// Wprowadź robota w stan STOPPED

}
// Wewnętrzna funkcja do zmiany kierunku silnika
// Przyjmuje zadany kierunek obrotu silnika
static void DRV8834_SetDirection(StepMotor_t *stepMotor, StepMotorDirection_t dir)
{
	if((dir == __MOTOR_FWD) || (dir == __MOTOR_BACK))
	{
		HAL_GPIO_WritePin(stepMotor->step_motor_pins.DIR_PORT, stepMotor->step_motor_pins.DIR_PIN, dir);
		stepMotor->direction = dir;
	}
}
// Funkcja do zmiany prędkości silnika, przyjmuje wartość zadaną prędkości
void DRV8834_SetSpeed(StepMotor_t *stepMotor, int32_t speed)
{
	// Zmienne pomocnicze wartości do której zlicza licznik i aktualnej częstotliwości PWM
	uint32_t counter, freq;
	// Jeśli prędkość jest ujemna i aktualny kierunek jest prosto to zmień kierunek na wstecz
	if((speed < 0) && (stepMotor->direction == __MOTOR_FWD))
	{
		DRV8834_SetDirection(stepMotor, __MOTOR_BACK);
	}
	// Odwrotnie niż poprzednio
	else if((speed > 0) && (stepMotor->direction == __MOTOR_BACK))
	{
		DRV8834_SetDirection(stepMotor, __MOTOR_FWD);
	}
	// Jeśli prędkość jest ujemna to wyznacz jej wartość bezwzględną
	if(speed < 0)
	{
		speed = ABS(speed);
	}
	// Jeśli z jakiegoś powodu prędkość będzie większa niż ustawiony limit górny
	// to ustaw prędkość jako limit górny (maxSpeed)
	if(speed > maxSpeed)
	{
		speed = maxSpeed;
	}
	// Oblicz wymaganą częstotliwość PWM na podstawie stałych częstotliwości minimalnej i maks
	// oraz aktualnego mikrokroku (jako mnożnik) i zadanej prędkości
	freq = (speed * (stepMotor->microstep*(maxFrequency - DRV8834_MOT_MIN_FREQ))) / maxSpeed;
	// Obliczenie wymaganej wartości do której ma zliczać licznik aby realizować wyliczoną częstotliwość
#if USING_WHICH_TIMERS == APB1
	// Wyliczania na podstawie stałych wybranego procesora i timera
	if(freq != 0)
		counter = PCLK_PRESCALER*HAL_RCC_GetPCLK1Freq() / (stepMotor->PWM_timer->Init.Prescaler * freq);
	else
		counter = 0;	// zabezpieczenie przed dzieleniem przez zero
#elif USING_WHICH_TIMERS == APB2	// jeśli wybrany timer taktowany jest z szyny APB2 to wykorzystaj inne stałe
		counter = PCLK_PRESCALER*HAL_RCC_GetPCLK2Freq() / (stepMotor->PWM_timer->Init.Prescaler * freq);
#else
#error "Choose between APB1 timers or APB2 timers or reimplement code if using both"
#endif
	// Jeśli wartość aktualnego countera jest taki sam jak poprzednio to pomiń zmiane ustawień timera
	if(stepMotor->last_counter != counter)
	{
		// Jeśli counter jest zerowy to zatrzymaj timer ustawiając zerowe wartości
		if(counter == 0)
		{
			__HAL_TIM_SET_COUNTER(stepMotor->PWM_timer, 0);
			__HAL_TIM_SET_AUTORELOAD(stepMotor->PWM_timer, 0);
			__HAL_TIM_SET_COMPARE(stepMotor->PWM_timer, stepMotor->PWM_timerChannel, 0);
		}
		// Jeśli jest różny od 0 to ustaw wartość do której ma zliczać jako counter - 1 (-1 bo zaczynamy od 0)
		else
		{
			__HAL_TIM_SET_COUNTER(stepMotor->PWM_timer, 0);
			__HAL_TIM_SET_AUTORELOAD(stepMotor->PWM_timer, counter - 1);
			// zmiana stanu na niski w połowie zliczania, aby wypełnienie PWM było równe 50%
			__HAL_TIM_SET_COMPARE(stepMotor->PWM_timer, stepMotor->PWM_timerChannel, (counter/2) - 1);
		}
		// Przypisz aktualny counter do kolejnej iteracji wyliczeń timera
		stepMotor->last_counter = counter;
	}
}
// Wystartuj silniki (ustawiany stan BALANCING robota) z zadaną prędkością
void DRV8834_StartMotor(StepMotor_t *stepMotor, int32_t speed)
{
	state = BALANCING;

	DRV8834_SetSpeed(stepMotor, speed);

	HAL_TIM_PWM_Start(stepMotor->PWM_timer, stepMotor->PWM_timerChannel);
}
