#include <SoftwareSerial.h> // UART Library
#include <Wire.h> // I2C library
#include <SPI.h> // SPI library
#include <SD.h> // SD card library
#include <Adafruit_BMP085.h> // BMP 085 or 180 pressure sensor library
#include <Adafruit_MPU6050.h> // MPU6050 accelerometer sensor library
#include <Adafruit_Sensor.h> // Adafruit unified sensor library
#include <Adafruit_LSM9DS1.h> // Include LSM9DS1 library

#define rfSerial Serial1
#define gpsSerial Serial2

// Create the sensor objects
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); //Accelerometer
Adafruit_BMP085 bmp; //Pitot Tube Pressure Sensor

int bitLenthList[10] = {12,//Seconds since launch
                        1, //Sender Ident
                        12, //Altitude
                        12, //Latitude
                        12, //Lonitude
                        9, //Velocity
                        8, //OrientationX
                        8, //OrientationY
                        30, //Error Code Array
                        8 //Checksum
                        };//Represents the number of bits for each part of the data transmission

int errorCodes[30] = {}; //Check https://docs.google.com/document/d/138thbxfGMeEBTT3EnloltKJFaDe_KyHiZ9rNmOWPk2o/edit for error code list

void setup() {
  // Start hardware serial communication (for debugging)
  Serial.begin(9600);
  
  // Start RFD UART communication
  rfSerial.begin(57600);
  Serial.println("Software serial initialized.");
  
  // Initialize LSM9DS1 sensor
  if (!lsm.begin()) {
    errorCodes[19] = 1;//Set error code if initialization fails
  }

  if (!bmp.begin()) {
    errorCodes[17] = 1;//Set error code if initialization fails
  }

  // Setup sensor ranges
  setupSensor();  // Added missing semicolon here
}

void loop() {
  char incomingByte;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print(incomingByte);
    rfSerial.print(incomingByte);
  }
  if (rfSerial.available() > 0) {
    incomingByte = rfSerial.read();
    Serial.println(incomingByte);
    rfSerial.print(incomingByte);
  }
  
  // Read sensor data
  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);
  
  // Send data over RFD900 (via SoftwareSerial)
  rfSerial.print("ACCEL "); rfSerial.print(accel.acceleration.x, 2); rfSerial.print(",");
  rfSerial.print(accel.acceleration.y, 2); rfSerial.print(",");
  rfSerial.print(accel.acceleration.z, 2);

  rfSerial.print(" GYRO "); rfSerial.print(gyro.gyro.x, 2); rfSerial.print(",");
  rfSerial.print(gyro.gyro.y, 2); rfSerial.print(",");
  rfSerial.print(gyro.gyro.z, 2); rfSerial.print(" rad/s");

  rfSerial.print(" MAG "); rfSerial.print(mag.magnetic.x, 2); rfSerial.print(",");
  rfSerial.print(mag.magnetic.y, 2); rfSerial.print(",");
  rfSerial.print(mag.magnetic.z, 2); rfSerial.print(" gauss");

  rfSerial.print(" TEMP "); rfSerial.print(bmp.readTemperature()); rfSerial.print("C");
  
  rfSerial.print(" PRESS "); rfSerial.print(bmp.readPressure()); rfSerial.print("Pa");
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  rfSerial.print(" ALT = "); rfSerial.print(bmp.readAltitude()); rfSerial.print("m");
  rfSerial.println();

  delay(50);
}

// Select I2C Bus On I2C Multiplexer (if used)
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);            // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}

void setupSensor() {
  // 1.) Set accelerometer range:
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // Other ranges can be set as needed

  // 2.) Set gyroscope range:
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // Other ranges can be set as needed

  // 3.) Set magnetometer range:
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // Other gains can be set as needed
}
