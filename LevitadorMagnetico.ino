#include <stdint.h>
#include <stdbool.h>
#define PART_TM4C123GH6PM
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"


//Analog Inputs, Potentiometer and Hall Effect sensors.
int sensores = PD_3;
int ref = PE_2;

//------------------------------------------------------
//PID Variables
float Kp = 0.5;
float Ki = 0.01;
float Kd = 0.25;
//------------------------------------------------------

//PID Constants
float e = 0;
float ed = 0;
float e_old = 0;
float E = 0;
float E_old = 0;
float Actual_Value = 0;
int output = 0;

//Analog values
int analogVal = 0;
int Vref = 0;

//DAC Variables
byte DACSignal = B00000000;


void initTimer()
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);   // 32 bits Timer
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0Isr);    // Registering  isr
  ROM_TimerEnable(TIMER0_BASE, TIMER_A);
  ROM_IntEnable(INT_TIMER0A);
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void Timer0Isr(void)
{
  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear the timer interrupt
  DAC();
}

void setup()
{
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(40, OUTPUT);
  Serial.begin(9600);
  initTimer();
}


void loop()
{
  unsigned long ulPeriod;
  unsigned int Hz = 100;   // frequency in Hz
  ulPeriod = (SysCtlClockGet() / Hz) / 2;
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod - 1);
  while (1)
  {
    analogVal = analogRead(sensores);
    Vref = analogRead(ref);

    e = Vref - analogVal;
    ed = e - e_old;
    E = E_old + e;
    Actual_Value = Kp * e + Ki * E + Kd * ed;
    e_old = e;
    E_old = E;

    output = 0.127 * Actual_Value + 128;

    if (Actual_Value >= 1000) {
      output = 255;
    }
    if (Actual_Value <= -1000) {
      output = 0;
    }
    Serial.println(Actual_Value);

    DACSignal = byte(output);
  }
}

void DAC()
{
  digitalWrite(31, bitRead(DACSignal, 0));
  digitalWrite(33, bitRead(DACSignal, 1));
  digitalWrite(14, bitRead(DACSignal, 2));
  digitalWrite(15, bitRead(DACSignal, 3));
  digitalWrite(17, bitRead(DACSignal, 4));
  digitalWrite(38, bitRead(DACSignal, 5));
  digitalWrite(19, bitRead(DACSignal, 6));
  digitalWrite(40, bitRead(DACSignal, 7));
}
