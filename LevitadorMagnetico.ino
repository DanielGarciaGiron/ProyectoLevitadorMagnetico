//Universidad del Valle de Guatemala 
//Proyecto Levitador Magnetico
//Daniel Garcia, 14152
//Ricardo Galindo, 

//Librerias necesarias
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

//Analog Inputs, Potenciometro y sensores de efecto Hall.
int SensorH = 5;
int SensorL = 6;
int Ref = 7;
int Control_Voltage = 0;
int Signal_Ind = 0;
int Actual_Value = 0;
int Signal_Mag = 0;
int Vref = 0;

//Variables de salida
int controlVoltage = 0; 
byte DACSignal = B00000000;
const int lowestBit = 31;
const int highestBit = 39;
int currentBit = 0;

//Variables PID
uint8_t redLedState = LOW;
uint8_t tiempo = LOW;
int erroractual = 0;
int Erroractual = 0;
int Errorviejo = 0;
int errorviejo = 0;
int errorderivativo = 0;
int errortotal = 0;
int Pot = 0;
int sensorValue = 0;
int sensorValue1 = 0;
int Sensores = 0;
  
int Kd = 0;
int Kp = 2;
int Ki = 0;

void setup()
{
  //Timer y comunicacion serial
  Serial.begin(9600);  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, MAP_SysCtlClockGet() / 100000);
  TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1IntHandler);
  MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  MAP_TimerEnable(TIMER1_BASE, TIMER_A);

  //Poner pines del 31 a 38 como salidas digitales.
  for (int thisPin =lowestBit; thisPin <= highestBit; thisPin++) { 
    pinMode(thisPin, OUTPUT); 
  }

}

void loop()
{
  //Entrada
  Signal_Ind = analogRead(SensorH);
  Signal_Mag = analogRead(SensorL);
  Vref = analogRead(Ref);

  //PID
  erroractual = Vref - (Signal_Mag - Signal_Ind);
  errorderivativo = (erroractual-errorviejo) /(MAP_SysCtlClockGet() / 100000) ;
  Erroractual = (Errorviejo + erroractual)*(MAP_SysCtlClockGet() / 100000) ;
  errortotal = (erroractual*Kp + Erroractual*Ki + errorderivativo*Kd) * 4095/255;

  //Salida
  DACSignal = byte(controlVoltage);
  DAC();
}


void Timer1IntHandler() {
  MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  Pot = analogRead(Ref);
  sensorValue = analogRead(SensorH);
  sensorValue1 = analogRead(SensorL);
  Sensores = sensorValue-sensorValue1;
  errorviejo = erroractual;
  Errorviejo = Erroractual;
  erroractual = Pot - Sensores;
}

void DAC(){
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




