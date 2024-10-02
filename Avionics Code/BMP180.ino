#include <Wire.h>
#include <Adafruit_BMP085.h>  // Library for BMP180/BMP085 sensor

Adafruit_BMP085 bmp;  // Create BMP object

void setup() {
  // Initialize serial communication with RFD900 (via UART)
  Serial.begin(57600);  // Set baud rate according to RFD900 configuration
  
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize the BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed!");
    while (1);  // Loop forever if sensor initialization fails
  }

  // Small delay for initialization
  delay(100);
}

void loop() {
  // Read temperature and pressure from BMP180
  float temperature = bmp.readTemperature();
  int32_t pressure = bmp.readPressure();

  // Send the data over the serial port (to RFD900)
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" *C, Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");

  // Delay before sending the next set of data
  delay(1000);  
}
