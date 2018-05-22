#include <stdint.h>
#include <stdbool.h>
#define PART_TM4C123GH6PM
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"


//Analog Inputs, Potentiometer and Hall Effect sensors.
int sensorH = 27;
int sensorL = 28;
int ref = 29;

//PID Variables
int Kp = 0;
int Ki = 0;
int Kd = 0;
int e = 0;
int ed = 0;
int e_old = 0;
int E = 0;
int E_old = 0;
int Actual_Value = 0;

//Analog values
int Signal_Mag = 0;
int Signal_Ind = 0;
int Vref = 0;

//DAC Variables
byte DACSignal = B00000000;
const int lowestBit = 31;
const int highestBit = 39;
int currentBit = 0;

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
  for (int thisPin =lowestBit; thisPin <= highestBit; thisPin++)
  { 
    pinMode(thisPin, OUTPUT); 
  }
  //Serial.begin(9600);
  initTimer();
}


void loop()
{
  unsigned long ulPeriod;
  unsigned int Hz = 100;   // frequency in Hz  
  ulPeriod = (SysCtlClockGet() / Hz)/ 2;
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A,ulPeriod -1);
  while (1)
  {
    Signal_Ind = analogRead(sensorH);
    Signal_Mag = analogRead(sensorL);
    Vref = analogRead(ref);

    e = Vref - (Signal_Mag - Signal_Ind);
    ed = e - e_old;
    E = E_old + e;
    Actual_Value = Kp*e + Ki*E + Kd*ed;
    e_old = e;
    E_old = E;
  }
}

void DAC()
{
    for (int index = 0; index <= 7; index++) { 
      currentBit = bitRead(DACSignal, index);
      if(currentBit == 0){
        digitalWrite(index+31, LOW);
      }
      else{
        digitalWrite(index+31, HIGH);
        }
    }
}


