#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
byte data[32] = {0};


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
    int i=0;
    while(Wire.available()) {
     data[i] = Wire.read();
     Serial.print(data[i]);
     Serial.print(' ');
     i++;
    }
    //Serial.println(' ');
    i--;
    if (data[(i-1)] == 0){
      number=data[i]+5;
    }
    if (data[(i-1)] == 1){
      number=data[i]+10;
    }
}

// callback for sending data
void sendData(){
    Wire.write(number);
}
