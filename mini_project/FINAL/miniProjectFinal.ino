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


float kp = 0.62;//0.6275;
float ki = 0.19;//0.024, increased a lot cuz we want wind up -> ensures we don't stop before the position we want -> increase for better transient response
float integrator = 0;
float e = 0;
float ts = 0;
float tc = millis();
float r = 0; 
float y = 0;
int u = 0; //controller (volts)


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
      y=myEnc.read();
      y = (y/1600)*PI;
      e=r-y;//difference in position or error
      //Serial.print(y);
      //Serial.print("\t");
      integrator = integrator + ts*e; //might need to divide by 1000
      u = kp*e+ki*integrator*e; //controller(voltage)
      Serial.println(u);
     
      u = u/7.9 *255; //this may be funky but shouldn't matter too much
      //check if u is overflowing (greater or less than 255)
      if(u > 255){
        u=255;
      }else if(u < -255){
        u = 255;
      }
      analogWrite(M1PWM, abs(u));//255 is 5V
      
      if(u<0){
        digitalWrite(M1DIR, LOW);
      }else{
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
