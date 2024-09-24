#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

// Create the sensor object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Define I2C addresses
#define LSM9DS1_XG_ADDRESS 0x6B  // Accelerometer and Gyroscope address
#define LSM9DS1_MAG_ADDRESS 0x1E // Magnetometer address

void setupSensor() {
  // 1.) Set accelerometer range:
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // lsm.LSM9DS1_ACCELRANGE_4G;
  // lsm.LSM9DS1_ACCELRANGE_8G;
  // lsm.LSM9DS1_ACCELRANGE_16G;

  // 2.) Set gyroscope range:
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // lsm.LSM9DS1_GYROSCALE_500DPS;
  // lsm.LSM9DS1_GYROSCALE_2000DPS;

  // 3.) Set magnetometer range:
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // lsm.LSM9DS1_MAGGAIN_8GAUSS;
  // lsm.LSM9DS1_MAGGAIN_12GAUSS;
  // lsm.LSM9DS1_MAGGAIN_16GAUSS;
}

void setup() {
  Serial.begin(115200);
  
  if (!lsm.begin()) {
    Serial.println("Failed to initialize LSM9DS1!");
    while (1);
  }
  Serial.println("LSM9DS1 Found!");

  // Setup sensor ranges
  setupSensor();
}

void loop() {
  // Read and display accelerometer data
  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

  // Read and display gyroscope data
  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" rad/s");

  // Read and display magnetometer data
  Serial.print("Mag X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(mag.magnetic.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(mag.magnetic.z); Serial.println(" gauss");

  // Read and display temperature
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" °C");

  Serial.println();
  delay(1000);  // Wait for 1 second before the next loop
}
