#include <SoftwareSerial.h>


SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {
  // put your setup code here, to run once:
  // Start hardware serial communication (for debugging)
  Serial.begin(9600);
  
  // Start software serial communication
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
