#include <SoftwareSerial.h> //UART Library
#include <Wire.h> //I2C library
#include <SPI.h> //SPI library
#include <SD.h> //SD card library
#include <Adafruit_BMP085.h> //BMP 085 or 180 pressure sensor library
#include <Adafruit_MPU6050.h> //MPU6050 accelerometer sensor library
#include <Adafruit_Sensor.h> //Adafruit unified sensor library

SoftwareSerial mySerial(2, 3); // UART to RFD 900

void setup() {
  // Start hardware serial communication (for debugging)
  Serial.begin(9600);
  
  // Start RFD UART communication
  mySerial.begin(57600);
  Serial.println("Software serial initialized.");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available()) {
    char incomingByte = mySerial.read();
    Serial.print("Received: ");
    Serial.println(incomingByte);
  }

  if (Serial.available()) {
    char outgoingByte = Serial.read();
    mySerial.print(outgoingByte);
  }
}

// Select I2C Bus On I2C Multiplexer
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}

