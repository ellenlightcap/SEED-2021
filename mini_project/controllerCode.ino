
//Create an Arduino program to perform a closed loop step response experiment. 
//In this experiment, the setpointshould initially be zero, and at a specified time the setpoint is changed to 1 rad. 
//The Arduino should display the current time and wheel position in a format that can be copy and pasted in to excel or Matlab, and then plotted.
#include <Encoder.h>
Encoder myEnc(2, 6);

#define D2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define SF 12


float kp = 62.75;
float ki = 24.69;
float integrator = 0;
float e_past = 0;
float e = 0;
float ts = 0;
float tc = millis();
float r = 1600; //pi
float y = 0;
float u = 0; //controller (volts)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);

  pinMode(D2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR,OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(SF, INPUT);
  digitalWrite(D2,HIGH);
  
  myEnc.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
      y=myEnc.read();
      e=r-y;//difference in position or error
      integrator = integrator + ts*e;
      u = kp*e+ki*integrator; //controller
      analogWrite(M1PWM, u);//255 is 5V
      digitalWrite(M1DIR, HIGH);
      ts = millis() - tc;
      tc = millis();
}
