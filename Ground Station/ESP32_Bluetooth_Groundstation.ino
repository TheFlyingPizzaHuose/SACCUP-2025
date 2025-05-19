 /*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.15.01 or later version;
     - for iOS 1.12.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <math.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "Starlance_Groundstation"
#define REMOTEXY_ACCESS_PASSWORD "Apollo11"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)   
uint8_t RemoteXY_CONF[] =   // 87 bytes
  { 255,0,0,205,0,80,0,19,0,0,0,0,24,2,106,200,200,84,1,1,
  5,0,67,2,10,102,4,4,4,191,10,70,135,26,41,67,2,16,102,4,
  4,16,191,10,70,135,26,41,67,2,22,102,4,4,28,191,10,70,135,26,
  41,67,2,28,102,4,4,40,191,10,70,135,26,41,67,2,34,102,4,4,
  52,191,10,70,135,26,41 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // output variables
  char value_01[41]; // string UTF8 end zero
  char value_02[41]; // string UTF8 end zero
  char value_03[41]; // string UTF8 end zero
  char value_04[41]; // string UTF8 end zero
  char value_05[41]; // string UTF8 end zero

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define LED 2

HardwareSerial teensySerial(1);  // Use UART1 (you can also use 2)

int bitLengthList[13] = {12,//Seconds since launch
                        1, //Sender Ident
                        12, //Altitude
                        32, //Latitude
                        32, //Lonitude
                        9, //Velocity
                        8, //OrientationX
                        8, //OrientationY
                        32, //Error Code Array
                        6, //Even bit Array
                        8, //Checksum
                        8, //Letter R
                        8 //Letter L
                        };//Represents the number of bits for each part of the data transmission excluding checksum and endchar
const int bitArrayLength = 176;
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
    LSM9SD1_FAIL = 28,
    INA219_FAIL = 29;


void setup() 
{
  RemoteXY_Init (); 
  teensySerial.begin(19200, SERIAL_8N1, 16, 17);
  Serial.begin(19200);
  pinMode(LED,OUTPUT);
  // TODO you setup code
}

char lastData = 0;
int message_index = 0;
int msg_recieved = false;

//RSSI Variables
char last[3] = {0, 0, 0};
char recorded[7] = {0,0,0,0,0,0,0};
int record = 0;
void loop() 
{ 
  RemoteXY_Handler ();
  
  if(teensySerial.available()){
    static char charArray[30] = {};
    digitalWrite(LED,HIGH);
    char data = teensySerial.read();
    Serial.print(data);
    //ESP Data
    if(last[0] == 'I' && last[1] == ':' && last[2] == ' '){
      record = 1;
    }
    last[0] = last[1];
    last[1] = last[2];
    last[2] = data;

    if(record>0 && data == ' '){

      for(int i = 0; i<record; i++){
        RemoteXY.value_01[i] = recorded[7-record+i];
        Serial.print(recorded[7-record+i]);
      }
      Serial.println();
      record = 0;
    }
    if(record > 0){
      record++;
    }
    recorded[0] = recorded[1];
    recorded[1] = recorded[2];
    recorded[2] = recorded[3];
    recorded[3] = recorded[4];
    recorded[4] = recorded[5];
    recorded[5] = recorded[6];
    recorded[6] = data;

    //Copy of code from teensy
    if(message_index > sizeof(charArray)){
      message_index = 0;
    }
    charArray[message_index] = data;
    if(data == 'L' && lastData == 'R'){//Checks if end characters are present
      if(message_index == 21){
        msg_recieved = true;
        for(int i = 0; i<22; i++){
          int* bins = uint_to_binary(charArray[i]);
          for(int x = 0; x<8; x++){
            bitArray[i*8 + x] = *(bins+x);
          }
        }
      }
      if(message_index == 4){
        byte temp = charArray[0];
        String line5 = "SV:" + String(temp) + " Vol: " + String(1.0*charArray[1]/10) + " mA: " + String(1*charArray[2]);
        char buffer[41] = {};
        line5.toCharArray(buffer, sizeof(buffer));
        strcpy(RemoteXY.value_05, buffer);
      }
      message_index = 0;
    }else{
      message_index++;
    }
    lastData = data;
    digitalWrite(LED,LOW);
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
      if(i == 3 || i == 4){
        uint32_t temp = (uint32_t)my_sum;
        memcpy(&telemetry[i], &temp, sizeof(float));
      }else{
        telemetry[i] = my_sum;
      }
      bit_arr_read_pos+=bitLengthList[i];
    }
    String line3 = "LAT:" + String(telemetry[3]) + " LON: " + String(telemetry[4]);
    char buffer[41] = {};
    line3.toCharArray(buffer, sizeof(buffer));
    strcpy(RemoteXY.value_03, buffer);
    String line4 = "DIR_X: " + String(telemetry[6]) + " DIR_Y: " + String(telemetry[7]);
    line4.toCharArray(buffer, sizeof(buffer));
    strcpy(RemoteXY.value_04, buffer);
    for(int i = 0; i < 13; i++){
      switch(i){
        case 0: break;
        case 1: break;
        case 2:{
          String str = "ALT: " + String(telemetry[i]);
          char buffer[41] = {};
          str.toCharArray(buffer, sizeof(buffer));
          strcpy(RemoteXY.value_02, buffer);
          break;
        }
        case 3: break;
        case 4: break;
        case 5: break;
        case 6: break;
        case 7: break;
        case 8://Deal with error Codes
          //printErr(); 
          break;
        case 9: //Event
          break;
        /*case 10: Serial.print("Time"); Serial.print(telemetry[i]);break;
        case 11: Serial.print("Time"); Serial.print(telemetry[i]);break;
        case 12: Serial.print("Time"); Serial.print(telemetry[i]);break;*/
      }
    }
    msg_recieved=false;
  }
  // TODO you loop code
  // use the RemoteXY structure for data transfer
  // do not call delay(), use instead RemoteXY_delay() 
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
      case ADXL345_FAIL + 32: Serial.print("ADXL375_FAIL  "); break;
      case LSM9SD1_FAIL + 32: Serial.print("LSM9SD1_FAIL  "); break;
      case INA219_FAIL + 32: Serial.print("INA219_FAIL  "); break;
    }
  }
  Serial.print(" ");
}