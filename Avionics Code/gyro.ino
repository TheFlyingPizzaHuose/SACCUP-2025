#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <SoftwareSerial.h>

// Create the sensor object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Define I2C addresses
#define LSM9DS1_XG_ADDRESS 0x6B  // Accelerometer and Gyroscope address
#define LSM9DS1_MAG_ADDRESS 0x1E // Magnetometer address

// Create SoftwareSerial object for RFD900 communication
SoftwareSerial rfSerial(2, 3);  // RX, TX

// Define the baud rate for RFD900
#define RADIO_BAUD 57600

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
  
  // Start RFD900 communication
  rfSerial.begin(RADIO_BAUD);
  
  // Initialize LSM9DS1 sensor
  if (!lsm.begin()) {
    Serial.println("Failed to initialize LSM9DS1!");
    while (1);
  }
  Serial.println("LSM9DS1 Found!");

  // Setup sensor ranges
  setupSensor();
}

void loop() {
  // Read sensor data
  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);

  // Display data to Serial Monitor
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" rad/s");

  Serial.print("Mag X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(mag.magnetic.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(mag.magnetic.z); Serial.println(" gauss");

  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" °C");
  
  // Send data over RFD900 (via SoftwareSerial)
  rfSerial.print("Accel X:"); rfSerial.print(accel.acceleration.x, 2); rfSerial.print(",");
  rfSerial.print("Y:"); rfSerial.print(accel.acceleration.y, 2); rfSerial.print(",");
  rfSerial.print("Z:"); rfSerial.print(accel.acceleration.z, 2); rfSerial.println(" m/s^2");

  rfSerial.print("Gyro X:"); rfSerial.print(gyro.gyro.x, 2); rfSerial.print(",");
  rfSerial.print("Y:"); rfSerial.print(gyro.gyro.y, 2); rfSerial.print(",");
  rfSerial.print("Z:"); rfSerial.print(gyro.gyro.z, 2); rfSerial.println(" rad/s");

  rfSerial.print("Mag X:"); rfSerial.print(mag.magnetic.x, 2); rfSerial.print(",");
  rfSerial.print("Y:"); rfSerial.print(mag.magnetic.y, 2); rfSerial.print(",");
  rfSerial.print("Z:"); rfSerial.print(mag.magnetic.z, 2); rfSerial.println(" gauss");

  rfSerial.print("Temperature:"); rfSerial.print(temp.temperature); rfSerial.println(" °C");

  // Add a delay for readability
  delay(1000);  // 1-second delay between readings
}
