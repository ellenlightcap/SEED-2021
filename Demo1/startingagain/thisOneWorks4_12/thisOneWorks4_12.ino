#include <Encoder.h>
#include <Wire.h>
Encoder myEnc1(6, 2); //right wheel
Encoder myEnc2(3, 5); // left wheel

#define D2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define SF 12

int period = 50;

float start_time = 0;
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
float Ki_rho = 0.73;
float I_rho_dot = 0; // forward velocity integrator
float I_rho = 0; // forward position integrator
float rho_dot_setpoint = 5; // in/s
float rho_setpoint=0;
bool POSITION_CONTROL = false;//if true then use rho_setpoint
// Controller Parameters
// angular velocity controller
float Kp_phi_dot = 20;  // PWM counts per rad/s error
float Ki_phi_dot = 200;
float Kp_phi = 14.12;
float Kd_phi = 0;
float Ki_phi = 0.79; 
float I_phi_dot=0; // angular velocity integrator
float I_phi=0; // angular position integrator
float phi_dot_setpoint = .1; // rad/s
float phi_setpoint=0;
bool ANGULAR_POSITION_CONTROL = false;//false for getting it to move at a speed, true for set angle

//I2C Globals
#define SLAVE_ADDRESS 0x04
int number = 0;
unsigned long longValue = 0;
float distance = 0;
float angle = 90;
byte data[32] = {0};
byte newdata[32] = {0};
int i=0;
bool interruptFlag=true;

int state = 3;
float distanceDiff = 0;
float angleDiff = 0;
 

void setup() {
  // put your setup code here, to run once:
  start_time = millis();
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
  //phi_setpoint = (rho)/(24);
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
  if (state == 2){
    Serial.print(phi);
  }
  phi += phi_dot*((float)period/1000.0); 
   
  //Serial.print("Angle");
  //Serial.print("\t");
  //Serial.println(angle);
  Controller();
  static float captureAngle;
  switch(state){
    case 1 ://spin until finding the marker
      if(angle != 90){
        phi_dot_setpoint = 0;
        //captureAngle = angle;
        //delay(2000);
        
        Serial.print("hi");
        Serial.print(captureAngle);
        Serial.println("\t");
      }
        //delay(1000);
        if (phi_dot_setpoint == 0){
          interruptFlag = 0;
          //delay(500);
          interruptFlag = 1;
          Serial.println("ey");
          //myEnc1.write(0);
          //myEnc2.write(0);
          captureAngle = angle;
          phi = 0;
          state = 2;
          
        }
        //state = 2;
      
      
        break;
    case 2 ://angle correction
      Serial.println("State 2!");
      Serial.println(phi_setpoint);
      Serial.println(phi);
      Serial.println(captureAngle);
      //myEnc1.write(0);
      //myEnc2.write(0);
      //ANGULAR_POSITION_CONTROL = true;
      //phi_setpoint = captureAngle;
      //POSITION_CONTROL = true;
      //ANGULAR_POSITION_CONTROL = true;
      //captureAngle = angle;
      //if(captureAngle < (-.01) || captureAngle > (.01)){
        //myEnc1.write(0);
        //myEnc2.write(0);
        ANGULAR_POSITION_CONTROL = true;
        Serial.println("Correcting...");
        phi_setpoint = captureAngle;
       
      //}
      
    
        break;
    case 3 ://go forward (account for 1 foot distance and distance of camera


      phi_dot_setpoint = 0; // rad/s
      phi_setpoint=0;
      ANGULAR_POSITION_CONTROL = false;
      POSITION_CONTROL = true; //will let us go a certain distance forward
      rho_setpoint= distance - 6;//in inches this will be where we put the distance from the beacon minus the numbers we need to 
      Serial.println(rho);
      Serial.println(rho_setpoint);
      distanceDiff = rho_setpoint - rho;
      if (distanceDiff < 0.35){
        delay(100);
        state = 4;
      }
      
      
      //Controller();
      //delay(1000);
      //state = 4;
      
      
      
      

        break;
    case 4 ://turn 90 degrees and do circle
      Serial.println("State 4!");
      POSITION_CONTROL = false;
      rho_setpoint = 0;
      rho_dot_setpoint = 0;
      ANGULAR_POSITION_CONTROL = true;
      phi_dot_setpoint = 0;
      phi_setpoint=-1.55;//should be 90 degrees
      Serial.println(phi);
      Serial.println(phi_setpoint);
      rho = 0;
      angleDiff = phi_setpoint - phi;
      if(angleDiff > -.05){
        //delay(200);

        //POSITION_CONTROL = false;
        //rho_dot_setpoint = 5; // in/s
        //rho_setpoint=0;
      
      


        //ANGULAR_POSITION_CONTROL = true;
      //rho = 0;

        //phi_setpoint = (rho)/(12);
        
        //phi_dot_setpoint = 0;
        //Serial.print("Phi: ");
        //Serial.print("\t");
        //Serial.println(phi);
        //delay(100);
        phi = 0;
        rho = 0;
        state = 5;
      }
      //delay(1000);
      //state = 5;
 
        break;
    case 5: //go in a circle 

      Serial.println("State 5!");
      //delay(100);
      POSITION_CONTROL = false;
      rho_dot_setpoint = 5; // in/s
      rho_setpoint=0;
//      
//      
//
//
      ANGULAR_POSITION_CONTROL = true;
//      //rho = 0;
//
      phi_setpoint = (rho)/(12);
      

      
        break;
        
    default:

        break;
  }
  // Controller Calculations
  
//  while(angle == 90){
//    Serial.print("while");
//    Serial.print("\t");
//    phi_setpoint = .524;
//    Serial.print("phi");
//    Serial.print("\t");
//    delay(2000);
//    Serial.print("delay");
//    Serial.print("\t");
//    
//    
//    if(angle != 90){
//      phi_setpoint = 0;
//    }
//    
//  }

  

  /*Serial.print(ANGULAR_POSITION_CONTROL);
  Serial.print("\t");
  Serial.print(POSITION_CONTROL);
  Serial.print("\t");
  Serial.print(rho_dot_setpoint);
  Serial.print("\t");
  Serial.print(rho_dot);
  Serial.print("\t");
  Serial.print(phi_dot_setpoint);
  Serial.print("\t");
  Serial.print(phi_dot);
  Serial.print("\t");
  Serial.print(rho_setpoint);
  Serial.print("\t");
  Serial.print(rho);
  Serial.print("\t");
  Serial.print(phi_setpoint);
  Serial.print("\t");
  Serial.print(phi);
  Serial.print("\t");
  */
  if (millis() > time_now + period){
    Serial.println("Error: main took longer than the desired time");
  }
  while(millis() < time_now + period){
    
  }
  //Serial.println();
  

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

   // set velocity setpoints to global variables in case velocity control is desired
   rho_dot_setpoint_internal = rho_dot_setpoint;
   phi_dot_setpoint_internal = phi_dot_setpoint;
   Ki_rho_dot_use = Ki_rho_dot;
   Ki_phi_dot_use = Ki_phi_dot;

   // implement outer controllers to set velocity setpoint, if position control is desired
   if (ANGULAR_POSITION_CONTROL) {
        phi_error = phi_setpoint - phi;
        I_phi += phi_error*(float)period/1000.0;
        phi_dot_setpoint= Kp_phi*phi_error + Kd_phi*(phi_error- phi_error_old)/((float)period/1000) + Ki_phi*I_phi;
        phi_error_old = phi_error;
        Ki_phi_dot_use = 0;
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

   PWM_bar = Kp_rho_dot*rho_dot_error + Ki_rho_dot_use*I_rho_dot;
   PWM_delta = Kp_phi_dot*phi_dot_error  + Ki_phi_dot_use*I_phi_dot;

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
  
 
  //Serial.print(M1PWM_value);
  //Serial.print("\t");
  //Serial.print(M2PWM_value);
  //Serial.print("\t");  
}

// callback for received data
void receiveData(int byteCount){
  if(interruptFlag){
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
      for(byte j = 1; j<5; j++) templongValue = (long(data[j]) << ((j-1)<<3)) | templongValue;
      distance = *((float*)&templongValue); // evil bit level hack
      Serial.println(distance);
    }else if(data[0] == 3){
      long templongValue = 0;
      for(byte j = 1; j<5; j++) templongValue = (long(data[j]) << ((j-1)<<3)) | templongValue;
      angle = (*((float*)&templongValue)) * (3.1415/180) ; // evil bit level hack
      Serial.println(angle);
    }
  }
}

// callback for sending data
void sendData(){
    if(data[0] == 64){
        Wire.write(number);
    //Long
    }else if(data[0] == 65){
      for(int j = 1; j < 5; j++) newdata[j] = byte(longValue & (0xFF << ((j-1) << 3)) >> ((j-1) << 3));
      Wire.write(newdata, 4);
    //Float
    }else if(data[0] == 66){
      //long tempLong = * ( long *) &floatValue;
      //for(int j = 1; j < 5; j++) newdata[j] = byte(tempLong & (0xFF << ((j-1) << 3)) >> ((j-1) << 3));
      Wire.write(newdata, 4);
    }
    
}
