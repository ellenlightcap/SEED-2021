#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
unsigned long longValue = 0;
float distance = 0;
float angle = 0;
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
    //data[0] = 195;
    //data[1] = 245;
    //data[2] = 72;
    //data[3] = 64; 
    //for(unsigned long j = 0; j<4; j++){
    //  Serial.println(data[j]);
    //  Serial.println((long(data[j]) << (j*8)));
    //  longValue = (long(data[j]) << (j<<3)) | longValue;
    //  Serial.println(longValue);
    //  delay(1000);
    //}
    
    //Serial.println(longValue);
    //floatValue = *((float*)&longValue);
    //Serial.println(floatValue);
    
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
      angle = *((float*)&templongValue); // evil bit level hack
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
      long tempLong = * ( long *) &floatValue;
      for(int j = 1; j < 5; j++) newdata[j] = byte(tempLong & (0xFF << ((j-1) << 3)) >> ((j-1) << 3));
      Wire.write(newdata, 4);
    }
    
}
