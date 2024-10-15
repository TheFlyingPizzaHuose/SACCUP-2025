/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)

Ground Station Main Code
Authors: Alleon Oxales, Ryan Santiago, Elizabeth McGhee

All pertinent information for this code is in document linked bellow.
This includes all references and credit to the authors of code that
is used in this program such as libraries.

Avionics Datasheet
https://docs.google.com/document/d/138thbxfGMeEBTT3EnloltKJFaDe_KyHiZ9rNmOWPk2o/edit
*/
#include <SoftwareSerial.h>
SoftwareSerial rfSerial(2, 3); // RX, TX pins for software serial

int bitLenthList[10] = {12,//Seconds since launch
                        1, //Sender Ident
                        12, //Altitude
                        12, //Latitude
                        12, //Lonitude
                        9, //Velocity
                        8, //OrientationX
                        8, //OrientationY
                        32, //Error Code Array
                        6 //Even bit Array
                        };//Represents the number of bits for each part of the data transmission excluding checksum and endchar
const int bitArrayLength = 112;
  const int charArrayLegnth = bitArrayLength/8 + 3;

void setup() {
    Serial.begin(57600); // Start serial communication at 57600 baud rate
    rfSerial.begin(57600);
    Serial.println("Ground Station Ready");
    int* bins = uint_to_binary('a');
}

char lastData = 0;
int index = 0;
void loop() {
  static char charArray[17] = {};
  if (rfSerial.available()) {
      char data = rfSerial.read(); // Read from software serial
      charArray[index] = data;
      Serial.print(data);
      if(data == 'L'){
        if(index == 16 && lastData == 'R'){
          for(int i = 0; i<17; i++){
            int* bins = uint_to_binary(charArray[i]);
            for(int x = 0; x<8; x++){
              //Serial.print(*(bins+x));
            }
          }
          Serial.println();
        }
        index = 0;
      }else{
        index++;
      }
      lastData = data;
      //
      //processData(data); // Process the received data
  }
  if (Serial.available()) {
      char data = Serial.read();
      rfSerial.print(data);
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


