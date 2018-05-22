#include <inc/tm4c123gh6pm.h>
//8000 PERIOD = 1 KILOHERTZ, can be modified.
#define PERIOD 80000
//PIN PF1 = RED_LED
#define LED RED_LED


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

void initTimer(void)
{
  SYSCTL_RCGCTIMER_R |= 1;
  TIMER0_CTL_R &= ~(1 << 8); // disable timer B
  TIMER0_CFG_R  = 4; 
  TIMER0_TBMR_R = 0b1010; // set T1AMS, T0MR=0
  TIMER0_TBILR_R = PERIOD & 0xffff; // set interval (lower 16 bits) 
  TIMER0_TBPR_R = PERIOD >> 16; // set interval (upper bits)
  TIMER0_TBMATCHR_R = 100; //  set match value
  TIMER0_CTL_R |= (1 << 8); // enable timer B
  GPIO_PORTF_AFSEL_R |= (1 << 1); // select alternate function for PF1 
  GPIO_PORTF_PCTL_R |= (7 << 4);
}

void ISR(void)
{
  DAC();
}

void setup()
{
  for (int thisPin =lowestBit; thisPin <= highestBit; thisPin++)
  { 
    pinMode(thisPin, OUTPUT); 
  }
  //Serial.begin(9600);
  pinMode(LED, OUTPUT); 
  initTimer();
  attachInterrupt(LED, ISR, RISING);
}


void loop()
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


