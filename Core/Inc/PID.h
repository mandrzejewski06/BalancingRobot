/*
 * PID.h
 *
 *  Created on: Jan 18, 2022
 *      Author: mand2
 */
#ifndef INC_PID_H_
#define INC_PID_H_

// Zmienić main.h na bibliotekę ze zmiennymi uint8_t itp.
#include "main.h"
// TODO: Dodać przedrostki PID do tych stałych
// Definicje stałych dla trybu AUTOMATIC i MANUAL
#define MANUAL	0		// Należy zaimplementować własną funkcję z algorytmem PID
#define AUTOMATIC	1	// Należy używać funkcji PID_Compute
// Definicje stałych dla kierunku obliczania wyjścia PID
#define DIRECT  0	// Im większe wejście tym większe wyjście
#define REVERSE  1	// Im większe wejście tym mniejsze wyjście
// Definicje stałych dla regulatora proporcjonalnego
#define P_ON_MEASUREMENT 0	// TODO: usunąć pOnM (kp*(-dInput))?
#define P_ON_ERROR 1		// kp*error
// Definicje stałych dla błędu i domyślnego okresu próbkowania
#define ERROR 0					// TODO: zaimplementować error jako enum
#define SAMPLE_TIME_DEFAULT 100	// Określa co ile będzie wywoływana funkcja PID_Compute
// Struktura dla obiektu PID
typedef struct PID {
	// Zmienne nastaw dla użytkownika
	// TODO: Zmienić nazwy na Kp, Ki, Kd
	float dispKp;				// wzmocnienie członu proporcjonalnego
	float dispKi;				// wzmocnienie członu całkującego
	float dispKd;				// wzmocnienie członu różniczkującego
	// Zmienne nastaw wewnętrznych dla biblioteki
	// TODO: zmienić nazwy na niemylące z tymi powyżej np. __prop, __intgrl, __deriv
	float kp;                  // j.w.
    float ki;                  // ki = ki * SampleTimeInSec
    float kd;                  // kd = kd / SampleTimeInSec
    // Zmienne do przechowywania kierunku obliczenia i rodzaju regulatora P
	uint8_t controllerDirection;
	uint8_t pOn;
	// Wskaźniki do zmiennych przechowujących wartości wejścia, wyjścia i setpointu
	// TODO: Rozważyć zmianę nazewnictwa zgodną ze schematem URA
    float *myInput;              // wejście regulatora
    float *myOutput;             // wyjście regulatora
    float *mySetpoint;           // setpoint
    // Zmienne wewnętrzne dla biblioteki
    // TODO: zmienić nazwę na np. __outputSum
	float outputSum, lastError; // zmienne przechowujące sumę członu I oraz wartość ostatniego uchybu
	// Pozostałe zmienne
	uint32_t SampleTime;	// Przechowuje okres próbkowania
	float outMin, outMax;	// Przechowuje minimalny i maksymalny zakres wyjścia regulatora
	bool inAuto, pOnE;		// Przechowuje stan trybu AUTOMATIC i trybu P_ON_ERROR
} PID_t;

// TODO: Dodać funkcje set i get -Kp -Ki oraz -Kd?
void setKp(PID_t *pid, float kp);
void setKi(PID_t *pid, float ki);
void setKd(PID_t *pid, float kd);
// Funkcja inicjalizująca obiekt PID
// TODO: Zmienić funkcję Init aby zamiast kierunek i pOn i ControllerDir przyjmowała sampleTime
void PID_Init(PID_t *pid, float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd, int POn, int ControllerDirection);
// Funkcja zmieniająca nastawy regulatora
// TODO: wyrzucić z tej funkcji POn
void PID_SetTunings(PID_t *pid, float Kp, float Ki, float Kd, int POn);
// Funkcja zmieniająca okres próbkowania
void PID_SetSampleTime(PID_t *pid, int NewSampleTime);
// Funkcja zmieniająca zakres wartości wyjściowych regulatora
void PID_SetOutputLimits(PID_t *pid, float Min, float Max);
// Funkcja zmieniająca tryb regulatora
void PID_SetMode(PID_t *pid, int Mode);
// Funkcja zmieniająca kierunek obliczania regulatora
void PID_SetControllerDirection(PID_t *pid, int Direction);
// Główna funkcja służąca do obliczania wyjścia regulatora, powinna być wywoływana co okres podanego wcześniej SampleTime
bool PID_Compute(PID_t *pid);

#endif /* INC_PID_H_ */
