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
#include <RH_RF95.h> // Include RFM9X library

//RFM9x pin assignments
#define RFM95_CS    10
#define RFM95_INT  9
#define RFM95_RST  14
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
const int RFM9X_PWR = 23;

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

int errorCodes[32] = {}; //Check document for error code list
//Error codes
const int PRGM_ERR = 0,
    ALT_OUT_RANGE = 1,
    LAT_OUT_RANGE = 2,
    LONG_OUT_RANGE = 3,
    VEL_OUT_RANGE = 4,
    ORIENT_X_OUT_RANGE = 5,
    ORIENT_Y_OUT_RANGE = 6,
    BMP280_ERR_DAT = 7,
    SAMM8Q_ERR_DAT = 8,
    MPU6050_ERR_DAT = 9,
    BMP180_1_ERR_DAT = 10,
    BMP180_2_ERR_DAT = 11,
    AS5600_1_ERR_DAT = 12,
    AS5600_2_ERR_DAT = 13,
    TELEM_PWR_FAULT = 14,
    VIDEO_PWR_FAULT = 15,
    MAIN_PWR_FAULT = 16,
    BMP280_FAIL = 17,
    SAMM8Q_FAIL = 18,
    MPU6050_FAIL = 19,
    BMP180_1_FAIL = 20,
    BMP180_2_FAIL = 21,
    AS5600_1_FAIL = 22,
    AS5600_2_FAIL = 23,
    SD_FAIL = 24,
    RFD900_FAIL = 25,
    RFM9X_FAIL = 26,
    ADXL345_FAIL = 27,
    LSM9SD1_FAIL = 28;

void setup() {
    Serial.begin(57600); // Start serial communication at 57600 baud rate
    rfSerial.begin(57600);
    Serial.println("Ground Station Ready!");

  //RFM9x start
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  while (!rf95.init()) {Serial.println("RFM9X_FAIL");}
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(RFM9X_PWR, false);
}

char lastData = 0;
int message_index = 0;
int last_time = millis();
int msg_recieved = false;
void loop() {
  static char charArray[17] = {};
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      msg_recieved = true;
      for(int i = 0; i<17; i++){
        int* bins = uint_to_binary(buf[i]);
        for(int x = 0; x<8; x++){
          bitArray[i*8 + x] = *(bins+x);
          //Serial.print(bitArray[i*8 + x]);
        }
      }
      //RH_RF95::printBuffer("Received: ", buf, len);
      //Serial.print("Got: ");
      //Serial.print("  RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);

      /* Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED_BUILTIN, LOW);*/
    }
  }
  if (false && rfSerial.available()) {
    char data = rfSerial.read(); // Read from software serial
    charArray[message_index] = data;
    if(data == 'L' && lastData == 'R'){//Checks if end characters are present
      if(message_index == 16){
        msg_recieved = true;
        for(int i = 0; i<17; i++){
          int* bins = uint_to_binary(charArray[i]);
          for(int x = 0; x<8; x++){
            bitArray[i*8 + x] = *(bins+x);
            //Serial.print(bitArray[i*8 + x]);
          }
        }
        message_index = 0;
      }
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
        if(i == 8){
          errorCodes[31 - my_index] = bitArray[bit_arr_read_pos+x];
        }
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
          Serial.print("  ");
          printErr(); 
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
      if(data == 'F'){
          data = 0xff;
          rfSerial.print(data);
      }else if(data == 'h'){
        printCommands();
      }else{
        for(int i = 0; i < 10; i++){
          data = data - '0';
          rfSerial.print(data);
        }
      }
  }
}

void printCommands(){
  Serial.println();
  Serial.println("Below are the commands for the avionics system:");
  Serial.println("1: Low Power Mode\n2: Normal Power Mode\n3: High Data Transfer");
  Serial.println("4: Low Data Transfer\n5: Redo Startup Sequence\n6: Report battery voltage");
  Serial.println("7: Ready for launch\n8: Toggle Debug\nF: Shutdown");
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


void printErr(){
  for(int i = 0; i < 32; i++){
    int check = i + errorCodes[i]*32;
    switch(check){
      case PRGM_ERR+32:Serial.print("PRGM_ERR  ");break;
      case ALT_OUT_RANGE+32:Serial.print("ALT_OUT_RANGE  ");break;
      case LAT_OUT_RANGE+32:Serial.print("LAT_OUT_RANGE  ");break;
      case LONG_OUT_RANGE + 32: Serial.print("LONG_OUT_RANGE  "); break;
      case VEL_OUT_RANGE + 32: Serial.print("VEL_OUT_RANGE  "); break;
      case ORIENT_X_OUT_RANGE + 32: Serial.print("ORIENT_X_OUT_RANGE  "); break;
      case ORIENT_Y_OUT_RANGE + 32: Serial.print("ORIENT_Y_OUT_RANGE  "); break;
      case BMP280_ERR_DAT + 32: Serial.print("BMP280_ERR_DAT  "); break;
      case SAMM8Q_ERR_DAT + 32: Serial.print("SAMM8Q_ERR_DAT  "); break;
      case MPU6050_ERR_DAT + 32: Serial.print("MPU6050_ERR_DAT  "); break;
      case BMP180_1_ERR_DAT + 32: Serial.print("BMP180_1_ERR_DAT  "); break;
      case BMP180_2_ERR_DAT + 32: Serial.print("BMP180_2_ERR_DAT  "); break;
      case AS5600_1_ERR_DAT + 32: Serial.print("AS5600_1_ERR_DAT  "); break;
      case AS5600_2_ERR_DAT + 32: Serial.print("AS5600_2_ERR_DAT  "); break;
      case TELEM_PWR_FAULT + 32: Serial.print("TELEM_PWR_FAULT  "); break;
      case VIDEO_PWR_FAULT + 32: Serial.print("VIDEO_PWR_FAULT  "); break;
      case MAIN_PWR_FAULT + 32: Serial.print("MAIN_PWR_FAULT  "); break;
      case BMP280_FAIL + 32: Serial.print("BMP280_FAIL  "); break;
      case SAMM8Q_FAIL + 32: Serial.print("SAMM8Q_FAIL  "); break;
      case MPU6050_FAIL + 32: Serial.print("MPU6050_FAIL  "); break;
      case BMP180_1_FAIL + 32: Serial.print("BMP180_1_FAIL  "); break;
      case BMP180_2_FAIL + 32: Serial.print("BMP180_2_FAIL  "); break;
      case AS5600_1_FAIL + 32: Serial.print("AS5600_1_FAIL  "); break;
      case AS5600_2_FAIL + 32: Serial.print("AS5600_2_FAIL  "); break;
      case SD_FAIL + 32: Serial.print("SD_FAIL  "); break;
      case RFD900_FAIL + 32: Serial.print("RFD900_FAIL  "); break;
      case RFM9X_FAIL + 32: Serial.print("RFM9X_FAIL  "); break;
      case ADXL345_FAIL + 32: Serial.print("ADXL345_FAIL  "); break;
      case LSM9SD1_FAIL + 32: Serial.print("LSM9SD1_FAIL  "); break;
    }
  }
  Serial.print(" ");
}
