#include <Wire.h>

#define SLAVE_ADDRESS 0x04
byte number = 0;
int state = 0;
byte data[32] = {0};


#include <Encoder.h>
Encoder myEnc(2, 6);

#define D2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define SF 12


float kp = 5;//pwm/rad
float ki = 0.18;//pwm/(rad*sec)
float integrator = 0; //radians*seconds
float e = 0; //radians
float ts = 0; //milliseconds
float tc = millis();
float r = 0; //radians
float y = 0; //radians but first encoder counts
int u = 0; //controller (volts/pwm)


void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  //Serial.println("Ready!");

  pinMode(D2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR,OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(SF, INPUT);
  digitalWrite(D2,HIGH);
  
  myEnc.write(0);

  Serial.println("Ready!");
  
}

void loop() {
      y=myEnc.read(); //encoder counts
      y = (y/1600)*PI; // radians
      //Serial.println(y); //testing
      
      //steady state error
      e=r-y;//difference in position/steady state error(radians)
      
      //Serial.print(y); //testing
      //Serial.print("\t"); //testing
      
      //integrator
      integrator = integrator + (ts/1000)*e; //radians*second
      
      //implementing controller
      u = kp*e+ki*integrator; //pwm
      u = u/7.9 * 255; //pwm with respect to the supplied voltage
      //Serial.println(u);//testing
      
      //check if u is overflowing (greater or less than 255)
      if(u > 255){
        u=255;
        analogWrite(M1PWM, u);//255 is 5V
        digitalWrite(M1DIR, HIGH); //one way
      }else if(u<0){
        u = u*-1;
        if(u > 255){
          u=255;
        }
        analogWrite(M1PWM, u);//if u is less than 0 go the other direction
        digitalWrite(M1DIR, LOW);
      }
      else{
        analogWrite(M1PWM, u);//positive and CW
        digitalWrite(M1DIR, HIGH);
      }
      ts = millis() - tc;
      tc = millis();
      //Serial.print(tc);
      //Serial.print("\t");
      //Serial.println();
}

// callback for received data
void receiveData(int byteCount){// This function reads in data from the PI to the arduino
    int i=0; 
    
    while(Wire.available()) {
     data[i] = Wire.read(); //This line takes the data from the wire to the arduino
     if(data[i]==5)break; //For this protocol in order to not get ISR timeouts, if the value is 5, this means the value being sent is for the read request, therfore we don't want to document anything
     //Serial.print(data[i]);//These print functions are for troubleshooting. These functions with 
     //Serial.print(' ');
     i++;
    }
    //Serial.println(' ');
    i--;
    if(data[i]!= 5 && i < 31 && i > 0){ //If we overflowed, underflowed, or got our incorrect offset value, then we don't update the r value
      Serial.print("<- New data ");
      r=data[i]*(PI/2); //This converts the value read in into radians and updates it for the controller
      Serial.println(r);
    }
}

// callback for sending data
void sendData(){
  Serial.println("Sending...");
  Serial.print(y);
  /*
  Serial.print(' ');
  Serial.print(int((y*40)));
  Serial.print(' ');
  number=int((y*40));
  Serial.println(number);
  */
  //This if else statement gets the current radial position and sends it back to the PI. This method is a bit hamfisted to convert the float into a byte. This will need improving in future projects
  if (y < 0.3){
    number = 0;
  }else if(y > 0. && y < 2){
    number = 1;
  }else if( y < 4){
    number = 2;
  }else{
    number = 3;
  }
  Wire.write(number);
}
