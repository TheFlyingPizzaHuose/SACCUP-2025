#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX pins for software serial


void setup() {
    Serial.begin(115200); // Start serial communication at 57600 baud rate
    mySerial.begin(57600);
    Serial.println("Ground Station Ready");
    int* bins = uint_to_binary('a');
}

int index = 0;
void loop() {
  static char charArray[17] = {};
  if (mySerial.available()) {
      char data = mySerial.read(); // Read from software serial
      charArray[index] = data;
      if(data == 'E'){
        if(charArray[index-1] == 'A'){
          if(index == 16){
            for(int i = 1; i<18; i++){
              int* bins = uint_to_binary(charArray[i]);
              for(int x = 0; x<8; x++){
                Serial.print(*(bins+x));
              }
            }
            Serial.println();
          }
        }
        index = 0;
      }
      index++;
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

int* uint_to_binary(char character){
  static int result[8];
  result[0] = (character & (1 << 7)) > 0;
  result[1] = (character & (1 << 6)) > 0;
  result[2] = (character & (1 << 5)) > 0;
  result[3] = (character & (1 << 4)) > 0;
  result[4] = (character & (1 << 3)) > 0;
  result[5] = (character & (1 << 2)) > 0;
  result[6] = (character & (1 << 1)) > 0;
  result[7] = (character & (1 << 0)) > 0;
  return result;
}


