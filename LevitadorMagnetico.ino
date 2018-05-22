#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"



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

//Interrupt variables
volatile uint32_t g_ui32Counter = 0;

void setup()
{
  for (int thisPin = lowestBit; thisPin <= highestBit; thisPin++)
  {
    pinMode(thisPin, OUTPUT);
  }
  //Serial.begin(9600);

  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  // Configure Timer0B as a 16-bit periodic timer.
  TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  // Set the Timer0B load value to 1ms.
  TimerLoadSet(TIMER0_BASE, TIMER_B, SysCtlClockGet() / 1000);
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  // Enable the Timer0B interrupt on the processor (NVIC).
  IntEnable(INT_TIMER0B);
  // Enable Timer0B.
  TimerEnable(TIMER0_BASE, TIMER_B);
  // Enable processor interrupts.
  IntMasterEnable();
  // Configure the Timer0B interrupt for timer timeout.
  TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
  // Initialize the interrupt counter.
  g_ui32Counter = 0;
  uint32_t ui32PrevCount = 0;
  // Loop forever while the Timer0B runs.
  while (1)
  {
    // If the interrupt count changed, print the new value
    if (ui32PrevCount != g_ui32Counter)
    {
      ui32PrevCount = g_ui32Counter;
    }
  }
}

void Timer0BIntHandler()
{
  // Clear the timer interrupt flag.
  TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

  DAC();

}

void loop()
{
  Signal_Ind = analogRead(sensorH);
  Signal_Mag = analogRead(sensorL);
  Vref = analogRead(ref);

  e = Vref - (Signal_Mag - Signal_Ind);
  ed = e - e_old;
  E = E_old + e;
  Actual_Value = Kp * e + Ki * E + Kd * ed;
  e_old = e;
  E_old = E;
}

void DAC()
{
  for (int index = 0; index <= 7; index++) {
    currentBit = bitRead(DACSignal, index);
    if (currentBit == 0) {
      digitalWrite(index + 31, LOW);
    }
    else {
      digitalWrite(index + 31, HIGH);
    }
  }
}


