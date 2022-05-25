/*
 * PID.c
 *
 *  Created on: Jan 18, 2022
 *      Author: mand2
 */
#include "PID.h"

// ********** FUNKCJA INICJALIZUJĄCA ********** //
void PID_Init(PID_t *pid, float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd, int POn, int ControllerDirection)
{
	// Przypisanie wskaźników
    pid->myOutput = Output;		// wyjście
    pid->myInput = Input;		// wejście
    pid->mySetpoint = Setpoint;	// nastawa
    // Przypisanie zmiennych początkowych
    pid->inAuto = false;		// tryb manualny, TODO: zmienić na inAuto = true?
    pid->SampleTime = SAMPLE_TIME_DEFAULT;	// domyślny okres próbkowania
    PID_SetOutputLimits(pid, 0, 255);		// domyślny zakres wyjścia 0-255
    PID_SetControllerDirection(pid, ControllerDirection);	// przypisanie kierunku zliczania
    PID_SetTunings(pid, Kp, Ki, Kd, POn);	// przypisanie nastaw
}

// ********** FUNKCJA DO ZMIANY NASTAW ********** //
void PID_SetTunings(PID_t *pid, float Kp, float Ki, float Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return; // jeśli któraś z nastaw jest mniejsza od 0, przerwij funkcję

   // Przypsanie typu członu proporcjonalnego
   pid->pOn = POn;
   pid->pOnE = (POn == P_ON_ERROR);

   // Przypisanie do zmiennych dla użytkownika odpowiednich nastaw
   pid->dispKp = Kp;
   pid->dispKi = Ki;
   pid->dispKd = Kd;

   // Skalowanie nastaw - przypisanie do zmiennych używanych wewnątrz biblioteki przez algorytm PID_Compute
   float SampleTimeInSec = ((float)pid->SampleTime)/1000; // konwersja okresu próbkowania na sekundy
   pid->kp = Kp;	// człon proporcjonalny bez zmian
   pid->ki = Ki * SampleTimeInSec;	// człon całkujący przeskalowany razy czas próbkowania
   pid->kd = Kd / SampleTimeInSec;	// człon różniczkujący dzielony przez czas próbkowania

   // Jeśli kierunek zliczania jest odwrotny - nastawy otrzymują przeciwne znaki (ujemne)
  if(pid->controllerDirection == REVERSE)
   {
	  pid->kp = (0 - pid->kp);
	  pid->ki = (0 - pid->ki);
	  pid->kd = (0 - pid->kd);
   }
}

void setKp(PID_t *pid, float kp)
{
	if(kp < 0)
		return;

	pid->kp = kp;
	pid->dispKp = kp;

	if(pid->controllerDirection == REVERSE)
		pid->kp = (0 - pid->kp);
}

void setKi(PID_t *pid, float ki)
{
	if(ki < 0)
		return;

	pid->dispKi = ki;
	pid->ki = ki * ((float) pid->SampleTime / 1000.0);

	if(pid->controllerDirection == REVERSE)
		pid->ki = (0 - pid->ki);
}

void setKd(PID_t *pid, float kd)
{
	if(kd < 0)
		return;

	pid->dispKd = kd;
	pid->kd = kd / ((float) pid->SampleTime / 1000.0);

	if(pid->controllerDirection == REVERSE)
		pid->kd = (0 - pid->kd);
}

// ********** FUNKCJA DO ZMIANY CZASU PRÓBKOWANIA ********** //
void PID_SetSampleTime(PID_t *pid, int NewSampleTime)
{
	// Jeśli czas próbkowania nie jest dodatni to przerwij funkcję
   if (NewSampleTime > 0)
   {
	  // oblicz proporcję nowego i starego czasu próbkowania
      float ratio  = (float) NewSampleTime / (float) pid->SampleTime;
      // Przemnóż nastawy ki i kd zgodnie z nową proporcją
      pid->ki *= ratio;
      pid->kd /= ratio;
      // Przypisz nowy czas próbkowania do obiektu
      pid->SampleTime = (unsigned long) NewSampleTime;
   }
}

// ********** FUNKCJA DO ZMIANY ZAKRESU SYGNAŁU WYJŚCIOWEGO (STERUJĄCEGO) ********** //
void PID_SetOutputLimits(PID_t *pid, float Min, float Max)
{
   // Jeśli minimum jest większe od maximum to przerwij funkcję
   if(Min >= Max) return;
   // Przypisz nowe wartości zakresu do obiektu
   pid->outMin = Min;
   pid->outMax = Max;

   // Jeśli PID jest w trybie automatycznym (algorytm PID_Compute) to przeskaluj wyjście,
   // gdyby obecna wartość przekroczyła nowy zakres
   if(pid->inAuto)
   {
	   // Przeskaluj bezpośrednio wyjście regulatora
	   if(*(pid->myOutput) > pid->outMax) *(pid->myOutput) = pid->outMax;
	   else if(*(pid->myOutput) < pid->outMin) *(pid->myOutput) = pid->outMin;
	   // Przeskaluj bezpośrednio zliczoną sumę członu całkującego
	   if(pid->outputSum > pid->outMax) pid->outputSum= pid->outMax;
	   else if(pid->outputSum < pid->outMin) pid->outputSum= pid->outMin;
   }
}

// ********** FUNKCJA DO ZMIANY TRYBU REGULATORA ********** //
void PID_SetMode(PID_t *pid, int Mode)
{
	// Sprawdź czy nowy tryb to tryb automatyczny
    bool newAuto = (Mode == AUTOMATIC);
    // Jeśli tak i poprzedni tryb był manualny
    if(newAuto && !pid->inAuto)
    {
		// Przypisz obecną wartość wyjścia jako sumę członu całkującego
		pid->outputSum = *pid->myOutput;
		// Jako ostatnie wejście przypisz aktualne wejście
		pid->lastError = *pid->mySetpoint - *pid->myInput;
		// Jeśli suma członu całkującego przekracza któryś z zakresów to ogranicz tę sumę
		if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
		else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
		// Nie ma potrzeby ograniczać samego wyjścia, gdyż będzie ono wyliczone przy
		// następnym wywołaniu funkcji PID_Compute
    }
	// Przypisz nowy tryb do obiektu
	pid->inAuto = newAuto;
}

// ********** FUNKCJA DO ZMIANY KIERUNKU ZLICZANIA ********** //
void PID_SetControllerDirection(PID_t *pid, int Direction)
{
	// Jeśli regulator jest w trybie auto i nastąpiła zmiana kierunku zliczania
	if(pid->inAuto && (Direction != pid->controllerDirection))
	{
		// Zmień znak nastaw regulatora na przeciwny
		pid->kp = (0 - pid->kp);
		pid->ki = (0 - pid->ki);
		pid->kd = (0 - pid->kd);
	}
	// Przypisz nowy kierunek zliczania do obiektu
	pid->controllerDirection = Direction;
}

// ********** FUNKCJA GŁÓWNA OBLICZAJĄCA WYJŚCIE REGULATORA ********** //
bool PID_Compute(PID_t *pid)
{
	// Jeśli funkcja jest wywołana w trybie manualnym to przerwij ją
	if(!pid->inAuto)
	{
		return false;
	}

	// Stwórz zmienne tymczasowe
	float input = *pid->myInput;	// Przypisz wejście (w przypadku gdyby zmieniło się w trakcie działania algorytmu)
	float error = *pid->mySetpoint - input;		// Oblicz uchyb
	float dError = (error - pid->lastError);	// Oblicz różnicę uchybu obecnej i ostatniej iteracji
	// Oblicz sumę członu całkującego
	pid->outputSum+= (pid->ki * error);

	// Jeśli P jest w trybie P_ON_MEASUREMENT
	if(!pid->pOnE)
	{
		// od sumy członu całkującego odejmij wzmocnienie członu proporcjonalnego
		pid->outputSum-= pid->kp * dError;
	}
	// Ogranicz tak obliczoną sumę do zakresów wyjściowych
	if(pid->outputSum > pid->outMax) pid->outputSum= pid->outMax;
	else if(pid->outputSum < pid->outMin) pid->outputSum= pid->outMin;

	// Jeśli P jest w trybie P_ON_ERROR to oblicz wyjście członu proporcjonalnego
	float output;
	if(pid->pOnE) output = pid->kp * error;
	// W przeciwnym wypadku wyjście = 0 gdyż człon proporcjonalny był uwzględniony w sumie członu całkującego
	else output = 0;

	// Dodaj do wyjścia regulatora sumę oraz człon różniczkujący
	output += pid->outputSum + pid->kd * dError;

	// Jeśli wyjście wyszło poza zakres to ogranicz je
	if(output > pid->outMax) output = pid->outMax;
	else if(output < pid->outMin) output = pid->outMin;
	// Przypisz obliczoną wartość wyjścia do zmiennej użytkownika
	*pid->myOutput = output;
	// Zapamiętaj wartość obecnego uchybu to kolejnej iteracji
	pid->lastError = error;

	return true;
}

