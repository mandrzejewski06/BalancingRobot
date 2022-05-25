/*
 * DRV8834.h
 *
 *  Created on: 15 lut 2022
 *      Author: mand2
 */

#ifndef INC_DRV8834_H_
#define INC_DRV8834_H_
// Stałe konfiguracyjne dla wybranych timerów sprzętowych
// Są potrzebne do wyliczeń okresu PWM i w konsekwencji prędkości silników
#define USING_WHICH_TIMERS APB1	// z której szyny taktowane są wybrane timery
#define PCLK_PRESCALER 2 // wartość dzielnika wybranej szyny
// Stałe konfiguracyjne dla dobranych silników krokowych
// TOMBIT 42STH50 - 5VDC 1.58A 0.91Nm
#define DRV8834_MOTOR_STEP_PER_REVOLUTION	200	// liczba pełnych kroków na obrót
#define DRV8834_MOT_DEFAULT_MAX_FREQ		350 // maksymalna częstotliwość PWM - 400 [Hz] to 2 [obr/s] - przy rozdzielczości 200
#define DRV8834_MOT_MIN_FREQ				1   // minimalna częstotliwość PWM
#define DRV8834_MOT_DEFAULT_MAX_SPEED		50  // liczba stopni prędkości, żeby nie sterować bezpośrednio częstotliwością
// Typ wyliczeniowy przechowujący stan robota
typedef enum {
	STOPPED = 0, FORWARD, BACKWARD, LEFT, RIGHT, BALANCING
} RoborState_t;
// Struktura pinów GPIO wykorzystywanych przez silnik
typedef struct
{
	uint16_t DIR_PIN;		// wybór kierunku obrotu
	GPIO_TypeDef* DIR_PORT;
	uint16_t M0_PIN;		// piny do wybrania mikrokroku
	GPIO_TypeDef* M0_PORT;	// więcej w pliku DRV8834.c funkcja setMicrostep()
	uint16_t M1_PIN;
	GPIO_TypeDef* M1_PORT;
} StepMotorPins_t;
// Kierunek obrotu silnika, typ węwnetrzny dla biblioteki
// Nie używać poza biblioteką
typedef enum {__MOTOR_FWD = 0, __MOTOR_BACK = 1} StepMotorDirection_t;
// Struktura obiektu pojedynczego silnika krokowego
typedef struct
{
	StepMotorDirection_t direction;		// kierunek obrotu silnika
	StepMotorPins_t step_motor_pins;	// struktura pinów do których podłączony jest modul DRV8834

	TIM_HandleTypeDef *PWM_timer;		// Który timer realizuje PWM wysterowania silnika
	uint32_t PWM_timerChannel;			// Do którego kanału timera moduł jest podłączony

	uint8_t microstep;					// zmienna przechowująca aktualny mikrokrok silnika (1, 2, 8 itd..)
	uint32_t last_counter;				// zmienna pomocnicza przechowująca ostatnią wartość do jakiej zlicza licznik
} StepMotor_t;

// Funkcje obsługujące zmienną stanu robota (stop, balancing, forward itp...)
void DRV8834_setRobotState(uint16_t st);
uint16_t DRV8834_getRobotState(void);
// Funkcje obsługujące zmienne maksymalnej prędkości i częstotliwości PWM
void DRV8834_setMaxSpeed(uint16_t spd);
void DRV8834_setMaxFreq(uint16_t freq);
uint16_t DRV8834_getMaxSpeed(void);
uint16_t DRV8834_getMaxFreq(void);
// Funkcja inicjalizująca pojedynczy silnik
void DRV8834_Init(StepMotor_t *stepMotor, TIM_HandleTypeDef *htim, uint32_t channel);
// Osobna funkcja inicjalizująca strukturę pinów
void DRV8834_InitPins(StepMotor_t *stepMotor, uint16_t dirPin, GPIO_TypeDef* dirPort, uint16_t m0Pin, GPIO_TypeDef* m0Port, uint16_t m1Pin, GPIO_TypeDef* m1Port);
// Funkcja pomocnicza wewnętrzna do zmiany pinu z wyjścia w wejście lub na odwrót
// Jest potrzebna do ustawiania mikrokroku (3 stany: wysoki, niski (wyjście) i wysoka impedancja (wejście)
void DeinitializePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t Mode);
// Funkcja do zmiany mikrokroku, wymaga wyłączonych silników (stan robota STOPPED)
// Dozwolone mikrokroki to 1, 2, 4, 8, 16, 32 (co odpowiada 1/1, 1/2, 1/4 itd...)
uint8_t DRV8834_SetMicrostep(StepMotor_t *stepMotor, uint8_t microstep);
// Funkcja do zatrzymania silnika (stan STOPPED)
void DRV8834_StopMotor(StepMotor_t *stepMotor);
// Funkcja do zmiany prędkości silnika, przyjmuje wartości od 0 do MAX_SPEED
void DRV8834_SetSpeed(StepMotor_t *stepMotor, int32_t speed);
// Funkcja do wystartowania silników (musi być wywołana po resecie i po każdym wprowadzeniu w stan STOPPED
void DRV8834_StartMotor(StepMotor_t *stepMotor, int32_t speed);

#endif /* INC_DRV8834_H_ */
