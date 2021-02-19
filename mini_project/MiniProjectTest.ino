#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
int data[32] = {0};

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
    while(Wire.available()) {
     Serial.print(Wire.read());
     Serial.print(' ');  
    }
}

// callback for sending data
void sendData(){
    Wire.write(number);
}
