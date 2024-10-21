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

#define rfSerial Serial1

int bitLengthList[13] = {12,//Seconds since launch
                        1, //Sender Ident
                        12, //Altitude
                        12, //Latitude
                        12, //Lonitude
                        9, //Velocity
                        8, //OrientationX
                        8, //OrientationY
                        32, //Error Code Array
                        6, //Even bit Array
                        8, //Checksum
                        8, //Letter R
                        8 //Letter L
                        };//Represents the number of bits for each part of the data transmission excluding checksum and endchar
const int bitArrayLength = 136;
static int bitArray[bitArrayLength] = {};
const int charArrayLegnth = bitArrayLength/8;

void setup() {
    Serial.begin(57600); // Start serial communication at 57600 baud rate
    rfSerial.begin(57600);
    Serial.println("Ground Station Ready");
}

char lastData = 0;
int message_index = 0;
int last_time = millis();
int msg_recieved = false;
void loop() {
  static char charArray[17] = {};
  if (rfSerial.available()) {
      char data = rfSerial.read(); // Read from software serial
      charArray[message_index] = data;
      //Serial.print(data);
      if(data == 'L'){//Checks if end characters are present
        if(message_index == 16 && lastData == 'R'){
          msg_recieved = true;
          //Serial.println();
          for(int i = 0; i<17; i++){
            int* bins = uint_to_binary(charArray[i]);
            for(int x = 0; x<8; x++){
              bitArray[i*8 + x] = *(bins+x);
              //Serial.print(bitArray[i*8 + x]);
            }
          }
        }
        message_index = 0;
      }else{
        message_index++;
      }
      lastData = data;
  }
  if(msg_recieved){
    int bit_arr_read_pos = 0;
    static float telemetry[13] = {};
    for(int i = 0; i < 13; i++){//Ellie McGhee
      float my_sum = 0;
      int my_index = 0;
      for(int x = bitLengthList[i]-1; x>=0; x--){//Converts bits back into valueEllie McGchee
          my_sum = pow(2, my_index) * bitArray[bit_arr_read_pos+x]+ my_sum;
          my_index++;
      }
      telemetry[i] = my_sum;
      bit_arr_read_pos+=bitLengthList[i];
    }
    Serial.println();
    for(int i = 0; i < 13; i++){
      switch(i){
        case 0: Serial.print("T+: "); Serial.print(telemetry[i]);break;
        case 1: Serial.print(" ID: "); Serial.print(telemetry[i]);break;
        case 2: Serial.print(" ALT: "); Serial.print(telemetry[i]);break;
        case 3: Serial.print(" LAT: "); Serial.print(telemetry[i]);break;
        case 4: Serial.print(" LON: "); Serial.print(telemetry[i]);break;
        case 5: Serial.print(" VEL: "); Serial.print(telemetry[i]);break;
        case 6: Serial.print(" OREINT_X: "); Serial.print(telemetry[i]);break;
        case 7: Serial.print(" OREINT_Y: "); Serial.print(telemetry[i]);break;
        case 8://Deal with error Codes
          Serial.print(" ERROR-DETECTED"); 
          break;
        case 9: //Event
          Serial.print(" EVENT-PLACEHOLDER"); 
          break;
        /*case 10: Serial.print("Time"); Serial.print(telemetry[i]);break;
        case 11: Serial.print("Time"); Serial.print(telemetry[i]);break;
        case 12: Serial.print("Time"); Serial.print(telemetry[i]);break;*/
      }
    }
    msg_recieved=false;
  }
  
  last_time = millis();
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

int* uint_to_binary(char character){//Extracts bits from character: Alleon Oxaels
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
}//end of uint_to_binary


float binary_to_dec(int my_bit_size, int* my_arr){//Ellie McGchee
    float my_sum = 0;
    int my_index = 0;
    for (int i = my_bit_size-1; i >= 0; i--){
        my_sum = pow(2, my_index) * *(my_arr+i)+ my_sum;
        my_index++;
    }
    return my_sum;
}//end binary to decimal


