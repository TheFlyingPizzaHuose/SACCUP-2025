#include <Wire.h>

#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

void setup() {
  // Initialize serial communication with RFD900 (via UART)
  Serial.begin(57600);  // Set baud rate according to RFD900 configuration
  
  // Initialize I2C communication
  Wire.begin();
  
  // Small delay for initialization
  delay(100);
}

void loop() {
  // Read the raw angle from AS5600
  int angle = readRawAngle();

  // Send the angle data over the serial port (to RFD900)
  Serial.print("Angle: ");
  Serial.println(angle);

  // Delay to prevent spamming the transmission
  delay(500);  
}

// Function to read raw angle from AS5600
int readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_REG);  // Set the register to read raw angle
  Wire.endTransmission();
  
  // Request 2 bytes from AS5600
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    
    // Combine high and low byte to form a 12-bit result
    int angle = (highByte << 8) | lowByte;
    
    // Return the angle
    return angle;
  } else {
    // Return -1 if reading fails
    return -1;
  }
}
