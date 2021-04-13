
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
int state = 0;
unsigned long longValue = 0;
float distance = 0;
float angle = 90;
byte data[32] = {0};
byte newdata[32] = {0};
int i=0;

int period = 50;

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
float rho_setpoint=24;
bool POSITION_CONTROL = true;//if true then use rho_setpoint
// Controller Parameters
// angular velocity controller
float Kp_phi_dot = 20;  // PWM counts per rad/s error
float Ki_phi_dot = 200;
float Kp_phi = 10;
float Kd_phi = 0;
float Ki_phi = 20; 
float I_phi_dot=0; // angular velocity integrator
float I_phi=0; // angular position integrator
float phi_dot_setpoint = 0; // rad/s
float phi_setpoint=0;
bool ANGULAR_POSITION_CONTROL = true;
 

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
  phi_setpoint = (rho)/(24);
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
  phi += phi_dot*((float)period/1000.0);  

  // Controller Calculations
  Controller();

  Serial.print(ANGULAR_POSITION_CONTROL);
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
  
  if (millis() > time_now + period){
    Serial.println("Error: main took longer than the desired time");
  }
  while(millis() < time_now + period){
    
  }
  Serial.println();
  

}


// callback for received data
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
