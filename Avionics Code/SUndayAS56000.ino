#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX pins for software serial


void setup() {
    Serial.begin(57600); // Start serial communication at 57600 baud rate
    mySerial.begin(57600);
    Serial.println("Ground Station Ready");
}

void loop() {
  char charArray[14] = {};
  int index = 0;
  if (mySerial.available()) {
      char data = mySerial.read(); // Read from software serial
      index++;
      charArray[index] = data;
      Serial.println(index);
      if(data == '\n'){
        index = 0;
        Serial.println(charArray);
      }
      //
      //processData(data); // Process the received data
  }
  if (Serial.available()) {
      String data = Serial.readStringUntil('\n'); // Read incoming data until newline
  }
}

void processData(String data) {
    // Parse the incoming data
    if (data.startsWith("Angle: ")) {
        String angleStr = data.substring(7); // Extract the angle value (after "Angle: ")
        float angle = angleStr.toFloat(); // Convert to float
        Serial.print("Received Angle: ");
        Serial.println(angle); // Print the angle value
    } else {
        Serial.println("Unknown data: " + data); // Handle unexpected data
    }
}
