
#include <Encoder.h>
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
//int state = 0;
unsigned long longValue = 0;
float distance = 0;
float angle = 90;
byte data[32] = {0};
byte newdata[32] = {0};
int i=0;

int period = 50;

int state = 1;
float distanceDiff = 0;
float angleDiff = 0;
float circleDiff = 0;
float rotateDiff = 0;
int state2Time;

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
float phi_dot_setpoint = 0.2; // rad/s
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
  Serial.begin(115200); // start serial for output
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
  static float captureAngle;
  static float captureDistance;
  float forwardDistance = 0;
  switch(state){
    case 1 ://spin until finding the marker
      if(angle != 90){
        phi_dot_setpoint = 0;
        state1Time = millis();
        //captureAngle = angle;
        //delay(2000);
        
        //Serial.print("hi");
        //Serial.print(captureAngle);
        //Serial.println("\t");
      }
        //delay(1000);
        if (phi_dot_setpoint == 0){
          if(millis() - state1Time > 1000){
            Serial.println("ey");
            captureAngle = angle;
            phi = 0;
            state = 2;
            
          }
          //int interruptFlag = 0;
          //delay(500);
          //interruptFlag = 1;
          //Serial.println("ey");
          //myEnc1.write(0);
          //myEnc2.write(0);
          //captureAngle = angle;
          //phi = 0;
          //state = 2;
          
        }
        //state = 2;
      
      
        break;
    case 2 ://angle correction
      Serial.println("State 2!");
//      if(captureAngle < 0.06 || captureAngle > -0.03){
//        state = 3;
//      }
//      else{
//        ANGULAR_POSITION_CONTROL = true;
//        Serial.println("Correcting...");
//        phi_setpoint = captureAngle;
//        Serial.println(phi);
//        Serial.println(phi_setpoint);
//        rotateDiff = phi_setpoint - phi;
//        if(rotateDiff > -0.005){
//          Serial.println("Going to state 3");
//          phi = 0;
//          rho = 0;
//          if(millis() - state2Time > 2000){
//            state = 3;
//            myEnc1.write(0);
//            myEnc2.write(0);
//          }
//          else{
//            state2Time = millis();
//          }
//          //captureDistance = distance;
//          //Serial.println("Distance");
//          //Serial.println(captureDistance);
//          //state = 3;
//        }
//        
//      }
      //Serial.println(phi_setpoint);
      //Serial.println(phi);
      //Serial.println(captureAngle);
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
        Serial.println(phi);
        Serial.println(phi_setpoint);
        rotateDiff = phi_setpoint - phi;
        if(rotateDiff > -0.005){
          Serial.println("Going to state 3");
          phi = 0;
          rho = 0;
          if(millis() - state2Time > 2000){
            state = 3;
            myEnc1.write(0);
            myEnc2.write(0);
          }
          else{
            state2Time = millis();
          }
          //captureDistance = distance;
          //Serial.println("Distance");
          //Serial.println(captureDistance);
          //state = 3;
        }


//        if (rotateDiff < 0.05){
//          if(millis() - state2Time > 1000){
//            state = 3;
//            Serial.println("hi");
//          }
//        
//        //delay(100);
//        //state = 4;
//        }
//        else{
//          state2Time = millis();
//        }
        
       
      //}

//      if(millis() - state2Time > 1500){
//        if(rotateDiff < 0.05){
//          state = 3;
//        }
//      }
//      else{
//        state2Time = millis();
//      }
      
    
        break;
    case 3 ://go forward (account for 1 foot distance and distance of camera


      //rho_dot_setpoint = 0; // in/s
      //rho_setpoint=0;
      //POSITION_CONTROL = false;//if true then use rho_setpoint

      phi_dot_setpoint = 0; // rad/s
      phi_setpoint=0;
      ANGULAR_POSITION_CONTROL = false;
       //will let us go a certain distance forward
      captureDistance = distance;
      forwardDistance = captureDistance - 12;
      Serial.println(captureDistance);
      Serial.println("Forward distance");
      Serial.println(forwardDistance);
      rho_setpoint=forwardDistance;//in inches this will be where we put the distance from the beacon minus the numbers we need to 
      rho_dot_setpoint = 8;//in/s
      POSITION_CONTROL = false;
      Serial.println(rho);
      Serial.println(rho_setpoint);
      distanceDiff = rho_setpoint - rho;
      int state3Time;
      if (distanceDiff < 6){
        POSITION_CONTROL = true;
      }
      else{
        POSITION_CONTROL = false;
      }
      if (distanceDiff < 0.41){
        if(millis() - state3Time > 1000){
          Serial.println(rho);
          Serial.println(phi);
          phi = 0;
          rho = 0;
          state = 4;
          Serial.println("hi");
        }
        
        //delay(100);
        //state = 4;
      }
      else{
        state3Time = millis();
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
      float state4Time;
      if(angleDiff > -.05){
        if(millis() - state4Time > 1000){
          phi = 0;
          rho = 0;
          state = 5;
        }
      
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
        //phi = 0;
        //rho = 0;
        //state = 5;
      }
      else{
        state4Time = millis();
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

      //check rho to see if gone 2pi*3

      Serial.println(rho);
      Serial.println(phi);
      
      //Serial.println(rho_setpoint);
      circleDiff = 71.3 - rho;
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
    
        break;
        
        
    default:

        break;
  }

//  Serial.print(ANGULAR_POSITION_CONTROL);
//  Serial.print("\t");
//  Serial.print(POSITION_CONTROL);
//  Serial.print("\t");
//  Serial.print(rho_dot_setpoint);
//  Serial.print("\t");
//  Serial.print(rho_dot);
//  Serial.print("\t");
//  Serial.print(phi_dot_setpoint);
//  Serial.print("\t");
//  Serial.print(phi_dot);
//  Serial.print("\t");
//  Serial.print(rho_setpoint);
//  Serial.print("\t");
//  Serial.print(rho);
//  Serial.print("\t");
//  Serial.print(phi_setpoint);
//  Serial.print("\t");
//  Serial.print(phi);
//  Serial.print("\t");
  
  if (millis() > time_now + period){
    Serial.println("Error: main took longer than the desired time");
  }
  while(millis() < time_now + period){
    
  }
  //Serial.println();
  

}


// callback for received data
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
void receiveData(int byteCount){
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
      for(byte j = 1; j<5; j++) longValue = (long(data[j]) << ((j-1))<<3) | longValue;
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
