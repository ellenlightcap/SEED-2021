#include <Encoder.h>
#include <math.h> 
Encoder myEnc1(6, 2); //right wheel
Encoder myEnc2(3, 5); // left wheel

#define D2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define SF 12

#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
unsigned long longValue = 0;
float distance = 0;
float angle = 90;
byte data[32] = {0};
byte newdata[32] = {0};
int i=0;
bool i2cFlag = true;

int period = 50;

int state = 1;
float distanceDiff = 0;
float angleDiff = 0;
float circleDiff = 0;
float rotateDiff = 0;
int state2Time;
byte currentState = 0;

float startM1=0;
float startM2=0;
float finalM1=0;
float finalM2=0;
float difference1=0;
float difference2=0;
float angularVelocity1 = 0;
float angularVelocity2 = 0;
float angularPosition1 = 0;
float angularPosition2 = 0;
float time_now = 0;
float radius = 3;//inches
float rho_dot = 0;
float phi_dot = 0;
float rho = 0;
float phi = 0;
float distanceAcross = 12;//inches

// Controller Parameters
// Forward velocity controller
float Kp_rho_dot = 4;  // PWM counts per in/s error
float Ki_rho_dot = 40;
float Kp_rho = 16.445;
float Kd_rho = 0;
float Ki_rho = 5;
float I_rho_dot = 0; // forward velocity integrator
float I_rho = 0; // forward position integrator
float rho_dot_setpoint = 0; // in/s
float rho_setpoint=0;
bool POSITION_CONTROL = false;//if true then use rho_setpoint
// Controller Parameters
// angular velocity controller
float Kp_phi_dot = 20;  // PWM counts per rad/s error
float Ki_phi_dot = 200;
float Kp_phi = 10;
float Kd_phi = 0;
float Ki_phi = 20; 
float I_phi_dot=0; // angular velocity integrator
float I_phi=0; // angular position integrator
float phi_dot_setpoint = 0.26; // rad/s
float phi_setpoint=0;
bool ANGULAR_POSITION_CONTROL = false;
 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(D2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR,OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(SF, INPUT);
  digitalWrite(D2,HIGH);
  
  myEnc1.write(0);
  myEnc2.write(0);

  //I2C Setup

  pinMode(13, OUTPUT);
  //Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready!");

}

void loop() {
  // put your main code here, to run repeatedly:
  time_now = millis();

  // starting positions
  //phi_setpoint = (rho)/(12);
  startM1 = myEnc1.read();
  startM2 = -myEnc2.read();
  //Serial.print(startM1);
  //Serial.print("\t");
  //Serial.print(startM2);
  //Serial.print("\t");

  //differences
  difference1 = startM1 - finalM1;
  difference2 = startM2 - finalM2;
  //Serial.print(difference1);
  //Serial.print("\t");
  //Serial.print(difference2);
  //Serial.print("\t");

  //angular Positons
  angularPosition1 = (difference1*2*PI)/3200;
  angularPosition2 = (difference2*2*PI)/3200;
  //Serial.print(angularPosition1);
  //Serial.print("\t");
  //Serial.print(angularPosition2);
  //Serial.print("\t");

  //angular Velocity
  angularVelocity1 = (1000*angularPosition1)/period;
  angularVelocity2 = (1000*angularPosition2)/period;
  //Serial.print(angularVelocity1); //Right wheen
  //Serial.print("\t");
  //Serial.print(angularVelocity2);//Left wheel
  //Serial.print("\t");


  // final positions 
  finalM1 = startM1;
  finalM2 = startM2;

  


  

  rho_dot = radius * (angularVelocity1 + angularVelocity2) * 0.5;
  phi_dot = radius * (angularVelocity1 - angularVelocity2) / distanceAcross;
  rho += rho_dot*((float)period/1000.0);
//  if (state == 2){
//    Serial.print(phi);
//  }
  phi += phi_dot*((float)period/1000.0);  

  // Controller Calculations
  Controller();
  int state1Time = 0;
  int flagTime = 0;
  static float captureAngle;
  static float captureDistance;
  float forwardDistance = 0;
  float fudge;
  switch(state){
    case 1 ://spin until finding the marker
      //Serial.println("State 1");

      phi_dot_setpoint = 0.26; // rad/s
      phi_setpoint=0;
      ANGULAR_POSITION_CONTROL = false;
      rho_dot_setpoint = 0; // in/s
      rho_setpoint=0;
      POSITION_CONTROL = false;//if true then use rho_setpoint
      if(angle == 90){
        state1Time = millis();
      }
      else{
        phi_dot_setpoint = 0;
        Serial.println("Angle +1 90");
       
        //if (phi_dot_setpoint == 0){
          Serial.println("phi_dot = 0");
          Serial.println((millis() - state1Time));
          if((millis() - state1Time) > 6000){
              //captureDistance = distance;
              //fudge = 6 / captureDistance;
              captureAngle = angle - 0.35;
              
              phi = 0;
              rho = 0;
              state = 2;
            
          }
          else{
            //state1Time = millis();
          }
      }
      
      
        break;
    case 2 ://angle correction
      Serial.println("State 2");

        
        ANGULAR_POSITION_CONTROL = true;
        Serial.println("Correcting...");
        //captureAngle = angle;
        phi_setpoint = captureAngle;
        Serial.println(phi);
        Serial.println(phi_setpoint);
        rotateDiff = phi_setpoint - phi;
        if((rotateDiff > -0.005) && (rotateDiff < 0.005)){
          Serial.println("Going to state 3");
          //phi = 0;
          //rho = 0;
          if(millis() - state2Time > 7000){
            rho = 0;
            phi = 0;
            rho_setpoint = 0;
            phi_setpoint = 0;
            rho_dot = 0;
            Serial.println("Rho 1");
            Serial.println(rho);
            Serial.println("Phi");
            Serial.println(phi);
            
            state = 3;
            captureDistance = distance;
//            Serial.println("Rho 2");
//            Serial.println(rho);
            Serial.print("Distance");
            Serial.println(distance);
            //myEnc1.write(0);
            //myEnc2.write(0);
          }
          else{
            state2Time = millis();
          }

        }
      
    
        break;
    case 3 ://go forward (account for 1 foot distance and distance of camera


      Serial.println("Rho 1");
      Serial.println(rho);
      Serial.println("Phi");
      Serial.println(phi);
      phi_dot_setpoint = 0; // rad/s
      phi_setpoint=0;
      ANGULAR_POSITION_CONTROL = true;
       //will let us go a certain distance forward
      //captureDistance = distance;
      i2cFlag = false;
      forwardDistance = captureDistance - 10;
      Serial.println(captureDistance);
      Serial.println("Forward distance");
      Serial.println(forwardDistance);
      rho_setpoint=forwardDistance;//in inches this will be where we put the distance from the beacon minus the numbers we need to 
      rho_dot_setpoint = 24;//in/s
      POSITION_CONTROL = false;
      Serial.println(rho);
      Serial.println(rho_setpoint);
      Serial.println(phi);
      Serial.println(phi_setpoint);
      distanceDiff = rho_setpoint - rho;
      int state3Time;
      if (distanceDiff < 6){
        POSITION_CONTROL = true;
      }
      else{
        POSITION_CONTROL = false;
      }
      if (distanceDiff < 0.48){
        if(millis() - state3Time > 1000){
          Serial.println(rho);
          Serial.println(phi);
          phi = 0;
          rho = 0;
          state = 4;
          //state = 6;
          Serial.println("hi");
        }
        

      }
      else{
        state3Time = millis();
      }
      
      
  
      
      
      

        break;
    case 4 ://turn 90 degrees and do circle
      Serial.println("State 4!");
      POSITION_CONTROL = false;
      rho_setpoint = 0;
      rho_dot_setpoint = 0;
      ANGULAR_POSITION_CONTROL = true;
      phi_dot_setpoint = 0;
      phi_setpoint=-1.70;//should be 90 degrees
      Serial.println(phi);
      Serial.println(phi_setpoint);
      rho = 0;
      angleDiff = phi_setpoint - phi;
      float state4Time;
      if(angleDiff > -.05){
        if(millis() - state4Time > 1000){
          phi = 0;
          rho = 0;
          state = 5;
        }
      }
      else{
        state4Time = millis();
      }
 
        break;
    case 5: //go in a circle 

      Serial.println("State 5!");
      POSITION_CONTROL = false;
      rho_dot_setpoint = 19.5; // in/s
      rho_setpoint=0;
      ANGULAR_POSITION_CONTROL = true;
//      //rho = 0;
//
      phi_setpoint = (rho)/(18);

      //check rho to see if gone 2pi*3

      Serial.println(rho);
      Serial.println(phi);
      
      //Serial.println(rho_setpoint);
      circleDiff = 25 - rho;
      if(circleDiff < .4){
        //ANGULAR_POSITION_CONTROL = false;
        POSITION_CONTROL = true;
        
        rho = 0;
        rho_setpoint = 0;
        rho_dot_setpoint = 0;
        phi = 0;
        phi_setpoint = 0;
        phi_dot_setpoint = 0;
        Serial.println("Damn nice job");
        state = 6;
        
        break;
      }
      
      

      
        break;

    case 6:
        currentState++;
        angle = 90;
        distance = 0;
        captureDistance = 0;
        rho = float(0);
        i2cFlag = true;
        state = 1;
        pinMode(13, OUTPUT);
  
        Wire.begin(SLAVE_ADDRESS);

        Wire.onReceive(receiveData);
        Wire.onRequest(sendData);
        if (currentState == 7){
          state = 7;
        }
    
        break;
    case 7:

        break;
        
        
    default:

        break;
  }
  
  if (millis() > time_now + period){
    Serial.println("Error: main took longer than the desired time");
  }
  while(millis() < time_now + period){
    
  }
  //Serial.println();
  

}



/*
###I2C READ_ME###
For this i2c protocol, we recieve data that has been byte sliced. For a number under a byte in size, we don't really need to reconstruct
the number after we recieve it. However, the main data type being sent between the pi and the arduino is a float. In order to reassemble
the float, we first need to recontruct all the bytes into the correct order into a single address. Luckily, both systems are little endian,
so we don't have to worry about rearranging the bits of each byte. Once we have the long of all 4 bytes from the pi, converting it into a 
float is where the magic happens. The line with the 'evil bit level hack' is actually from the video game Quake III (which is now completely
open source). In essence we reference the address of the long as a float address then derefence it. This allows us to read the bits of the 
long as a float. The last step is converting units for the controller. The distance units are the same, but the angle must be converted from
degrees to radians. This is as simply a single linear factor. 
**NOTE**
The send data function has yet to be fully implemented. This is beacause our design doesn't have any data flow from the arduino to the pi, but
the process would be the same as the retrieve. 
*/

// callback for received data
void receiveData(int byteCount){
  
  if(i2cFlag){
    Serial.println("New Data!");
    i=0;
    while(Wire.available()) {
     //if(Wire.read() == 0) continue;
     data[i] = Wire.read();
     //Serial.print(data[i]);
     //Serial.print(' ');
     if(i == 0 && data[i] > 63) break;
     i++;
    }
    //Byte
    if(data[0] == 0){
        number = data[1];
        Serial.println(number);
    //Long
    }else if(data[0] == 1){
      longValue = 0;
      for(byte j = 1; j<5; j++) longValue = (data[j] << ((j-1))<<3) | longValue;
      Serial.println(longValue);
    //Float
    }else if(data[0] == 2){
      long templongValue = 0;
      distance=0;
      for(byte j = 1; j<5; j++) templongValue = (long(data[j]) << ((j-1)<<3)) | templongValue;
      distance = *((float*)&templongValue); // evil bit level hack
      //Serial.println(distance);
    }else if(data[0] == 3){
      long templongValue = 0;
      for(byte j = 1; j<5; j++) templongValue = (long(data[j]) << ((j-1)<<3)) | templongValue;
      angle = (*((float*)&templongValue)) * (3.1415/180) ; // evil bit level hack
      //Serial.println(angle);
    }
  }
}



void Controller() {
   float rho_dot_error=0;
   float phi_dot_error=0;
   float PWM_bar=0;
   float PWM_delta = 0;
   int M1PWM_value=0;
   int M2PWM_value=0;
   float rho_error=0;
   float rho_error_old = 0;
   float phi_error=0;
   float phi_error_old=0;
   float rho_dot_setpoint_internal=0;
   float phi_dot_setpoint_internal=0;
   float Ki_rho_dot_use = 0;
   float Ki_phi_dot_use = 0;
   float phi_dot_max=5;
   float Kp_phi_dot_use = 0;
   float Kp_rho_dot_use = 0;

   // set velocity setpoints to global variables in case velocity control is desired
   rho_dot_setpoint_internal = rho_dot_setpoint;
   phi_dot_setpoint_internal = phi_dot_setpoint;
   Ki_rho_dot_use = Ki_rho_dot;
   Ki_phi_dot_use = Ki_phi_dot;
   Kp_phi_dot_use = Kp_phi_dot;
   Kp_rho_dot_use = Kp_rho_dot;

   // implement outer controllers to set velocity setpoint, if position control is desired
   if (ANGULAR_POSITION_CONTROL) {
        phi_error = phi_setpoint - phi;
        I_phi += phi_error*(float)period/1000.0;
        phi_dot_setpoint= Kp_phi*phi_error + Kd_phi*(phi_error- phi_error_old)/((float)period/1000) + Ki_phi*I_phi;
        phi_error_old = phi_error;
        if (phi_dot_setpoint>phi_dot_max) {
          phi_dot_setpoint = phi_dot_max;
          I_phi=0;
        }
        if (phi_dot_setpoint<-phi_dot_max) {
          phi_dot_setpoint = -phi_dot_max;
          I_phi=0;
        }
        Ki_phi_dot_use = 0;
        Kp_phi_dot_use = 100;
   }
   if (POSITION_CONTROL) {
         rho_error = rho_setpoint - rho;
         I_rho += rho_error*(float)period/1000.0;
         rho_dot_setpoint = Kp_rho*rho_error + Kd_rho*(rho_error - rho_error_old)/((float)period/1000) + Ki_rho*I_rho;
         rho_error_old = rho_error;
         Ki_rho_dot_use = 0;
   }

   // Inner Control Loop
   rho_dot_error = rho_dot_setpoint - rho_dot;
   phi_dot_error = phi_dot_setpoint - phi_dot;

   I_rho_dot = I_rho_dot + rho_dot_error*(float)period/1000.0;
   I_phi_dot = I_phi_dot + phi_dot_error*(float)period/1000.0;

   PWM_bar = Kp_rho_dot_use*rho_dot_error + Ki_rho_dot_use*I_rho_dot;
   PWM_delta = Kp_phi_dot_use*phi_dot_error  + Ki_phi_dot_use*I_phi_dot;

   M1PWM_value = (PWM_bar+PWM_delta)/2;
   M2PWM_value = (PWM_bar-PWM_delta)/2;

  if (M1PWM_value>0) {
    digitalWrite(M1DIR,LOW);
  }
  else{
    digitalWrite(M1DIR,HIGH);
  }

  if (M2PWM_value>0) {
    digitalWrite(M2DIR,HIGH);
  }
  else{
    digitalWrite(M2DIR,LOW);
  }
  
  M1PWM_value = abs(M1PWM_value);
  M2PWM_value = abs(M2PWM_value);

  if (M1PWM_value>255) {
    M1PWM_value =255;
    I_rho_dot=0;
    I_phi_dot=0;
    I_phi=0;  
    I_rho=0;  
    }
  if (M2PWM_value>255) {
    M2PWM_value =255;
    I_rho_dot=0;
    I_phi_dot=0;
    I_phi=0;  
    I_rho=0;  
  }
  analogWrite(M1PWM, M1PWM_value);
  analogWrite(M2PWM, M2PWM_value); 
  
 
//  Serial.print(M1PWM_value);
//  Serial.print("\t");
//  Serial.print(M2PWM_value);
//  Serial.print("\t");  
}

void sendData(){
    Wire.write(currentState);
}
