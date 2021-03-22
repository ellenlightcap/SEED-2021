#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
long longNumber = 0;
float floatNumber = 0;
byte data[32] = {0};
byte newdata[32] = {0};
int i=0;

void setup() {
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
    delay(100);
}

// callback for received data
void receiveData(int byteCount){
    i=0;
    while(Wire.available()) {
     //if(Wire.read() == 0) continue;
     data[i] = Wire.read();
     //Serial.print(data[i]);
     //Serial.print(' ');
     if(i == 0 && data[i] > 126) break;
     i++;
     //stringSize++;
    }
    //Byte
    if(data[0] == 0){
        number = data[1];
    //Long
    }else if(data[0] == 1){
      longValue = 0;
      for(int j = i; j>=1; i--) longValue = (data[j] << ((j-1))<<3) | longValue;
    //Float
    }else if(data[0] == 2){
      longValue = 0;
      for(int j = i; j>=1; i--) longValue = (data[j] << ((j-1))<<3) | longValue;
      floatValue = * ( float *) &longValue; // evil floating point bit level hack
    }
    
}

// callback for sending data
void sendData(){
    //Serial.println(stringSize);
    //stringSize=1;
    //Wire.write(newdata,stringSize);
    if(data[0] == 64){
        Wire.write(number);
    //Long
    }else if(data[0] == 65){
      
      for(int j = 0; j < 4; i++) newdata[j] = (longValue & (0xFF << (i << 3)) >> (i << 3)
      Wire.write(newdata, 4);
    //Float
    }else if(data[0] == 66){
      long tempLong = * ( long *) &floatValue;
      for(int j = 0; j < 4; i++) newdata[j] = (tempLong & (0xFF << (i << 3)) >> (i << 3)
      Wire.write(newdata, 4);
    }
    
}
