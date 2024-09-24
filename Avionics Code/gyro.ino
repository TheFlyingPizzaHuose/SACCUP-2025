#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

// Create a SoftwareSerial object for RFD900 communication
SoftwareSerial rfSerial(2, 3);  // RX, TX

// Define the baud rate for the RFD900
#define RADIO_BAUD 57600

MPU6050 mpu;

void setup() {
  // Start the serial monitor for debugging
  Serial.begin(57600);
  
  // Start the SoftwareSerial for RFD900
  rfSerial.begin(RADIO_BAUD);
  
  // Initialize I2C communication and MPU-6050
  Wire.begin();
  mpu.initialize();
  
  // Check if the MPU-6050 is connected properly
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);  // Stop the program if the sensor isn't connected
  }
  
  Serial.println("MPU6050 connected!");
}

void loop() {
  // Variables to hold raw gyroscope data
  int16_t gx, gy, gz;
  
  // Get raw gyroscope data from MPU-6050
  mpu.getRotation(&gx, &gy, &gz);
  
  // Convert the raw gyro data to degrees/second
  float gyroX = gx / 131.0;  // 131 LSB/°/s sensitivity
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;
  
  // Print the gyro data to the Serial Monitor for debugging
  Serial.print("Gyro X: "); Serial.print(gyroX);
  Serial.print(" | Gyro Y: "); Serial.print(gyroY);
  Serial.print(" | Gyro Z: "); Serial.println(gyroZ);

  // Send the gyro data via RFD900 in the format "X:gyroX,Y:gyroY,Z:gyroZ"
  rfSerial.print("X:"); rfSerial.print(gyroX, 2);  // Sending X-axis with 2 decimal places
  rfSerial.print(",Y:"); rfSerial.print(gyroY, 2);  // Sending Y-axis with 2 decimal places
  rfSerial.print(",Z:"); rfSerial.println(gyroZ, 2);  // Sending Z-axis and end with newline

  // Small delay before the next reading
  delay(100);
}

