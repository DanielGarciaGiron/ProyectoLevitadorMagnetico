//Universidad del Valle de Guatemala 
//Proyecto Levitador Magnetico
//Daniel Garcia, 14152
//Ricardo Galindo, 



int SensorH = 5;
int SensorL = 6;
int Ref = 7;
int e = 0;
int ed = 0;
int Control_Voltage = 0;
int Kp = 0;
int Ki = 0;
int Kd = 0;
int e_old = 0;
int E_old = 0;
int Signal_Ind = 0;
int Actual_Value = 0;


void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  Kp = 1;
  Ki = 1;
  Kd = 1;

  Signal_Ind = analogRead(SensorH);
  Signal_Mag = analogRead(SensorL);
  Vref = analogRead(Ref);

  e = Vref - (Signal_Mag - Signal_Ind);
  ed = e - e_old;
  E = E_old + e;
  Actual_Value = Kp*e + Ki*E + Kd*ed;
  e_old = e;
  E_old = E;

  
  
  
}
