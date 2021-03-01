//Step Response Experiment to determine transfer function

//encoder
#include <Encoder.h>
Encoder myEnc(2, 6);

int period = 50;
unsigned long time_now = 0;


float finalPosition = 0;
float angularPosition = 0;
float startingPosition = 0;
float difference = 0;
float angularVelocity = 0;
float transferFunction = 0;

int initialVolt = 0;
int finalVolt = 5;

#define D2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define SF 12




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

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
    time_now = millis();
    Serial.print(time_now);
    Serial.print("\t");
    
    startingPosition = myEnc.read();
    Serial.print(startingPosition);
    Serial.print("\t");

    // the program will initially output a motor voltage command of 0, changing to a desired positive value at 1 second.
    //By motor voltage command, we mean the value used in the analogWritefunction)
    if(time_now < 1000){
      analogWrite(M1PWM, 0);
      digitalWrite(M1DIR, HIGH);
      Serial.print(initialVolt);
      Serial.print("\t");
      
    }
    if(time_now >= 1000){
      analogWrite(M1PWM, 255);//255 is 5V
      digitalWrite(M1DIR, HIGH);
      Serial.print(finalVolt);
      Serial.print("\t");
    }

    //read motor encoder and calculate the angular position (relative to the starting position)
    finalPosition = (myEnc.read());
    //Serial.println("Final: ");
    Serial.print(finalPosition);
    Serial.print("\t");
    
    difference = finalPosition - startingPosition;
    //Serial.println("difference: ");
    Serial.print(difference);
    Serial.print("\t");
    
    angularPosition = 0.0019634*difference; //amount of radians rotated: 0.0019634 = (2pi)/3200
    //Serial.println("angular Position: ");
    Serial.print(angularPosition);
    Serial.print("\t");
    
    angularVelocity = (1000*angularPosition)/period; //rad per sec -> 0.05 is 50 ms
    //Serial.println("angular velocity: ");
    Serial.print(angularVelocity);
    Serial.print("\t");
    

  

    if (millis() > time_now + period){
      Serial.println("Error: main took longer than the desired time");
    }
    while(millis() < time_now + period){
        //Serial.println("Amount of time to wait: ");
       // Serial.print(time_now+period-millis());
        //Serial.print("\t");
    }
    Serial.println();
    
}
