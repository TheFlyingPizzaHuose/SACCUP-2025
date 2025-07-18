/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)

Avionics Main Code
Authors: Alleon Oxales, Ryan Santiago, Elizabeth McGhee

All pertinent information for this code is in document linked bellow.
This includes all references and credit to the authors of code that
is used in this program such as libraries.

Avionics Datasheet
https://docs.google.com/document/d/138thbxfGMeEBTT3EnloltKJFaDe_KyHiZ9rNmOWPk2o/edit

Callibration spreadsheet
https://docs.google.com/spreadsheets/d/1Gov30G9uyXv7lDdadh1TLPG05m9mBo-JQmem5j_yBDY/edit?usp=sharing
*/
#include <SoftwareSerial.h>    // UART Library
#include <Wire.h>              // I2C library
#include <SPI.h>               // SPI library
#include <SD.h>                // SD card library
#include "Adafruit_BMP085.h"   // BMP 085 or 180 pressure sensor library
#include <Adafruit_BMP280.h>   // BMP 280 pressure sensor library
#include <Adafruit_MPU6050.h>  // MPU6050 accelerometer sensor library
#include <Adafruit_Sensor.h>   // Adafruit unified sensor library
#include <Adafruit_LSM9DS1.h>  // Include LSM9DS1 library
#include <Adafruit_ADXL375.h>  // Include ADXL375 library
#include "Adafruit_INA219.h"   // Include INA 219 library
#include <RH_RF95.h>           // Include RFM9X library
#include <cmath>
#include <iostream>
#include <EEPROM.h>
#include <uRTCLib.h>  //Include Real Time Clock Library
#include <time.h>
using namespace std;

uRTCLib rtc(0x68);  //Real time clock I2C address
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

#define BMP_SCL 13
#define BMP_SDO 12
#define BMP_SDA 11

//UART Serial Objects
#define rfSerial Serial1
#define gpsSerial Serial2

//RFM9x pin assignments
#define RFM95_CS 10
#define RFM95_INT 9
#define RFM95_RST 26
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
const int RFM9X_PWR = 23;
int16_t packetnum = 0;  // packet counter, we increment per xmission

const bool callibration_mode = 0;
const bool debug = 0;

//pin assignments
const int video_pin = 2;
const int shutdown_pin = 3;

//SD card variables
const int sdSelect = BUILTIN_SDCARD;
char* logFileName;
File logfile;

char* checkFile(bool mode = 0) {  //Alleon Oxales
  int fileExists = 1;
  int fileNum = 0;
  static char fileName[8] = "000.txt";
  while (fileExists) {
    fileName[0] = '0' + fileNum / 100;
    fileName[1] = '0' + (fileNum % 100) / 10;
    fileName[2] = '0' + fileNum % 10;
    File checkfile = SD.open(fileName);
    checkfile.seek(1 * 6);
    String content = checkfile.readStringUntil('\r');
    if (content != "") {
      fileNum++;
    } else {
      fileExists = 0;
    }
  }
  if (mode) {
    if (fileNum > 0) { fileNum--; }
    fileName[0] = '0' + fileNum / 100;
    fileName[1] = '0' + (fileNum % 100) / 10;
    fileName[2] = '0' + fileNum % 10;
  }
  return fileName;
}  //end checkFile

// Create the sensor objects
Adafruit_INA219 ina219; //INA 219 Object
Adafruit_MPU6050 mpu;         //Accelerometer
Adafruit_BMP280 bmp1(&Wire);  //Pitot Tube Pressure Sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(&Wire);
static sensors_event_t LSM_acc, LSM_gyro, LSM_mag, LSM_temp;
Adafruit_BMP085 bmp2;
Adafruit_BMP085 bmp3;
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345, &Wire1);

//GPS Variables
int gpsVersion = 2;  //SAM-M8Q
uint gps_sats = 0;
uint glonas_sats = 0;
uint gallileo_sats = 0;
uint8_t setGSV_ON[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}; //Left here so that it's checksum can be calculated during start
uint8_t setGSV_OFF[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };

// AV Modes
bool lowPower = 0;
bool videoPower = 0;
bool pwr_gps_status_poll = 1;
int time_vid_off = 0;
int shutdownCheck[3] = {0, 0, 0};  //All three elements must be >0 to activate shutdown
int restartCheck[3] = {0, 0, 0}; //All three elements must be >0 to activate restart
int time_last_command = 0;


int bitLengthList[10] = {
  12,  //Seconds since launch
  1,   //Sender Ident
  12,  //Altitude
  32,  //Latitude
  32,  //Longitude
  9,   //Velocity
  8,   //OrientationX
  8,   //OrientationY
  32,  //Error Code Array
  6    //Even bit Array
};     //Represents the number of bits for each part of the data transmission excluding checksum and endchar
const int bitArrayLength = 152;
const int charArrayLegnth = bitArrayLength / 8 + 3;
int my_event_arr[6] = {0,0,0,0,0,0};
int errorCodes[32] = {};  //Check document for error code list

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
          ADXL375_FAIL = 27,
          LSM9SD1_FAIL = 28,
          INA219_FAIL = 29,
          ACCEL_CALLIB = 30,
          MAG_CALLIB = 31;


//Component cycle time micros
uint BMP280_RATE = 5500,  //Ultra low: 5.5, Low: 7.5, Standard: 11.5, High: 19.5, Ultra High: 37.5
     SAMM8Q_RATE = 100000,
     MPU6050_RATE = 125,
     BMP180_RATE = 5500,  //Ultra low: 3, Standard: 5, High: 9, Ultra High: 17, Adv. High: 51
     AS5600_RATE = 150,
     RFD_RATE = 44983, //Prime number near 45ms
     RFM_RATE = 50000,
     LSM_RATE = 1500,
     ADXL345_RATE = 350,
     SD_RATE = 10000,
     DYN_START_RATE = 100000,
     STATUS_RATE = 10000;

//Component last cycle time micros
uint BMP280_LAST = 0,
     SAMM8Q_LAST = 0,
     MPU6050_LAST = 0,
     BMP180_LAST = 0,
     AS5600_LAST = 0,
     RFD_LAST = 0,
     RFM_LAST = 0,
     LSM_LAST = 0,
     ADXL345_LAST = 0,
     SD_LAST = 0,
     DYN_START_LAST = 0,
     STATUS_LAST = 0;

//Time variables
uint8_t time_launch = 0;
uint8_t last_millis = 0;
uint8_t last_micros = 0;
float time_since_launch = 0;

//Start Variables
float pos_start[3] = {0, 0, 0};
float quat_start[4] = {0, 0, 0, 0}; //start quaternion
float accel_start[3] = {0, 0, 0}; //start acceleration which measures gravity direction
float mag_start[3] = {0, 0, 0}; //start mag which measures gravity direction
float press_start = 101325;
int AoA_start1 = 0, AoA_start2 = 0;


//Telemetry Variables
int sender_indent = 1;  //0: Avionics 1: Payload drone
bool in_flight = 0;
float orientationX = 0,     //degrees
  orientationY = 0;     //degrees

//Sensor Data
float BMP280_PRESS = 0,
      BMP280_ALT = 0,
      GPS_LAT = 0,
      GPS_LON = 0,
      MPU_AX = 0,
      MPU_AY = 0,
      MPU_AZ = 0,
      MPU_GX = 0,
      MPU_GY = 0,
      MPU_GZ = 0,
      BMP180_1_PRESS = 0,
      BMP180_2_PRESS = 0,
      BMP180_1_ALT = 0,
      BMP180_2_ALT = 0,
      AS5600_1_ANG = 0,
      AS5600_2_ANG = 0,
      ADXL345_AX = 0,
      ADXL345_AY = 0,
      ADXL345_AZ = 0,
      LSM_AX = 0,
      LSM_AY = 0,
      LSM_AZ = 0,
      LSM_GX = 0,
      LSM_GY = 0,
      LSM_GZ = 0,
      LSM_MX = 0,
      LSM_MY = 0,
      LSM_MZ = 0;

//Kinematics variables
float latitude = 0,         //degrees,minutes,seconds,arcseconds
      longitude = 0;        //degrees,minutes,seconds,arcseconds
float position[3] = { 0, 0, 0 },  //Absolute position measurements
  velocity[3] = { 0, 0, 0 },      //Velocity measurements
  acc[3] = { 0, 0, 0 },      //Acceleration measurement
  abs_rot[4] = { 1, 0, 0, 0 },  //Absolute rotation measurements
  dt_rot[3] = { 0, 0, 0 },   //Derivative rotation measurements
  d2_rot[3] = { 0, 0, 0 };   //2nd Derivative rotation measurements

void setup() {
  Serial.begin(57600);    // Start hardware serial communication (for debugging)
  rfSerial.begin(57600);  //Init RFD UART

  if(SRC_SRSR != 1){setErr(PRGM_ERR);}// Read reset status register and PRGM_ERR if reset is not a power cycle
  //RFM9x start
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  if (!rf95.init()) { setErr(RFM9X_FAIL); }
  if (!errorCodes[RFM9X_FAIL]) {
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(RFM9X_PWR, false);
  }

  Serial.println("Initializing");
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  pinMode(video_pin, OUTPUT);
  pinMode(shutdown_pin, OUTPUT);

  URTCLIB_WIRE.begin();
  //rtc.set(0, 57, 19, 2, 17, 2, 25);// Set the time rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year) (1=Sunday, 7=Saturday)

  if (!SD.begin(sdSelect)) { setErr(SD_FAIL); }  //Init SD, this comes first to be able to check the latest log file
  if (detect_good_shutdown()) {
    normalStart();
  } else {//Sets all errors to start the dynamic start process.
    setErr(MAIN_PWR_FAULT);
    setErr(BMP280_FAIL);
    setErr(SAMM8Q_FAIL);
    setErr(MPU6050_FAIL);
    setErr(BMP180_1_FAIL);
    setErr(BMP180_2_FAIL);
    setErr(ADXL375_FAIL);
    setErr(LSM9SD1_FAIL);
    setErr(INA219_FAIL);
  }
  //gpsChecksum(poll_interfere, sizeof(poll_interfere));
  gpsChecksum(setGSV_ON, sizeof(setGSV_ON));
}

void pres_start_samples(float pres, bool reset = 0){//This function was moved up here because it needs to be in-context of loop() since it takes in input
  static int count = 0;
  static float average = 101325;
  static int start_time = 0;
  if(reset){
    count = 0;
    average = 101325;
    start_time = millis();
  }else if(millis()-start_time < 1000){//Only sample for 1000ms
    average = (average*count + pres)/(count+1);
    Serial.println(average);
    count++;
  }else{
    press_start = average;
  }
}

uint8_t last_gps_byte = 0;
bool record_bool = 0;
int record_count = 0;
int temp = 0;
void loop() {
  if (shutdownCheck[0] == 0xff && shutdownCheck[1] == 0xff && shutdownCheck[2] == 0xff) {  //Shutdown Mode
    digitalWrite(shutdown_pin, HIGH);
  } else if (!errorCodes[MAIN_PWR_FAULT]) {  //Normal Mode
    if (micros() < last_micros) {                         //Clock rollover check
      BMP280_LAST = 0;
      SAMM8Q_LAST = 0;
      MPU6050_LAST = 0;
      BMP180_LAST = 0;
      AS5600_LAST = 0;
      LSM_LAST = 0;
      SD_LAST = 0;
      STATUS_LAST = 0;
    }
    
    
    readRFD();
    readRFM();
    //At each sample, this also checks if the sensor has failed.
    //Serial.print(BMP280_ALT); Serial.print('|');
    //Serial.print(BMP180_1_ALT); Serial.print('|');
    //Serial.print(BMP180_2_ALT); Serial.println();
    time_since_launch = micros() / 1000000 - time_launch;
    static char gps_msg[200] = {};
    static int gps_msg_index = 0;
    if(gpsSerial.available() > 0){
      //temp = micros();
      int read_num = gpsSerial.available();
      for(int i = 0; i < read_num; i++){
        char incomingByte = gpsSerial.read();
        gps_msg[gps_msg_index] = incomingByte;
        //Serial.print(incomingByte);
        gps_msg_index++;
        if (incomingByte == '\n') {
          parseGPS(gps_msg, gps_msg_index);
          gps_msg_index = 0;
        }
        if(gps_msg_index > 190){gps_msg_index = 0;}//Prevent buffer overflow 
      }
      //Serial.println(micros() - temp);
    }else{
      if (micros() - AS5600_LAST > AS5600_RATE) {
        AS5600_1_ANG = readRawAngle(&Wire) - AoA_start2;
        AS5600_2_ANG = readRawAngle(&Wire1) - AoA_start2;
        if (AS5600_1_ANG < 0) {
          setErr(AS5600_1_FAIL);
        } else {
          clearErr(AS5600_2_FAIL);
        }
        if (AS5600_2_ANG < 0) {
          setErr(AS5600_2_FAIL);
        } else {
          clearErr(AS5600_2_FAIL);
        }
        AS5600_LAST = micros();
        if(!errorCodes[SD_FAIL]){
          logfile.print(micros());
          logfile.print("|6|");
          logfile.print(AS5600_1_ANG);
          logfile.print('|');
          logfile.println(AS5600_2_ANG);
        }
      }
      if (micros() - BMP280_LAST > BMP280_RATE && !errorCodes[BMP280_FAIL]) {
        BMP280_PRESS = bmp1.readPressure();
        BMP280_LAST = micros();
        if(!errorCodes[SD_FAIL]){
          logfile.print(micros());
          logfile.print("|1|");
          logfile.println(BMP280_PRESS);
        }
        float temp = bmpAplyCalibrat(0, BMP280_PRESS);
        pres_start_samples(temp, 0);
        BMP280_ALT = alt_from_pres(temp);//Adjust BMP pressure based on callibration after recording

      }
      if (micros() - BMP180_LAST > BMP180_RATE) {
        if(!errorCodes[BMP180_1_FAIL]){
          BMP180_1_PRESS = bmp2.readPressure();
          bmp2.readCommand();
        }
        if(!errorCodes[BMP180_2_FAIL]){
          BMP180_2_PRESS = bmp3.readPressure();
          bmp3.readCommand();
        }
        //position[2] = BMP180_2_PRESS;
        BMP180_LAST = micros();
        if(!errorCodes[SD_FAIL]){
          logfile.print(micros());
          logfile.print("|2|");
          logfile.print(BMP180_1_PRESS);
          logfile.print('|');
          logfile.println(BMP180_2_PRESS);
        }
        BMP180_1_ALT = alt_from_pres(bmpAplyCalibrat(1, BMP180_1_PRESS));//Adjust BMP pressure based on callibration after recording
        BMP180_2_ALT = alt_from_pres(bmpAplyCalibrat(2, BMP180_2_PRESS));
      }
      if (micros() - MPU6050_LAST > MPU6050_RATE && !errorCodes[MPU6050_FAIL]) {
        readMPU6050();
        MPU6050_LAST = micros();
      }
      if (micros() - LSM_LAST > LSM_RATE && !errorCodes[LSM9SD1_FAIL]) {
        readLSM();
        LSM_LAST = micros();
      }
      if (micros() - ADXL345_LAST > ADXL345_RATE && !errorCodes[ADXL375_FAIL]) {
        readADXL();
        ADXL345_LAST = micros();
      }
      if (micros() - SD_LAST> SD_RATE && !errorCodes[SD_FAIL]){
        logfile.flush();
      }
    }
    sendRFD();
    sendRFM();
    if(my_event_arr[0]){//Launch event autotriggers
      lowPower = 0;
      videoPower = 1;
      gpsSet(setGSV_OFF, sizeof(setGSV_OFF));
      pwr_gps_status_poll = 0;
    }
  } else {  //Recovery Mode
    dynamicStart();
    readRFD();
    readRFM();
    sendRFD();
    sendRFM();
  }
  //Low Power Mode
  if (lowPower) {  
    RFD_RATE = 1000000;
    sendRFD();
    sendRFM();
    readRFM();
    readRFD();
  }else{  
    RFD_RATE = 125;
  }
  //Video On/Off
  if(videoPower){
    digitalWriteFast(video_pin, HIGH);
  }else{
    int time_since_off = millis()-time_vid_off;
    if(time_since_off < 500){
      digitalWrite(video_pin, LOW);
    }else if(time_since_off > 500 && time_since_off < 2500){
      digitalWrite(video_pin, HIGH);
    }else if(time_since_off > 2500){
      digitalWrite(video_pin, LOW);
    }
  }
  //Power and sattilite status
  if(pwr_gps_status_poll && micros()-STATUS_LAST > STATUS_RATE){
    char busvoltage = 0, current_mA = 0;
    if(!errorCodes[INA219_FAIL]){
      busvoltage = std::round(ina219.getBusVoltage_V()*10);
      current_mA = abs(std::round(ina219.getCurrent_mA()));
    }
    //Send sat and power data
    char sat_count = gps_sats + glonas_sats + gallileo_sats;
    rfSerial.print(sat_count);
    rfSerial.print(busvoltage);
    rfSerial.print(current_mA);
    rfSerial.print('R');
    rfSerial.print('L');
    STATUS_LAST = micros();
  }
  
}

//==========START SEQUENCES==========Alleon Oxales
void normalStart() {
  Serial.println("NORM_START");
  //Initialize sensors and set error if it fails
  if (!initSAM_M8Q()) { setErr(SAMM8Q_FAIL); }
  if (!mpu.begin(104, &Wire1)) { setErr(MPU6050_FAIL); } else{setupMPU6050();}  //Setups MPU range of measurement and measurement rate
  if (!lsm.begin()) { setErr(LSM9SD1_FAIL); } else{setupLSM();}  //Setups LSM range of measurement and measurement rate
  if (!bmp1.begin()) { setErr(BMP280_FAIL); }  
  if (!bmp2.begin(0, &Wire2) /*0: low power, 1: normal, 2: high res, 3: ultra high*/) { setErr(BMP180_1_FAIL); }
  if (!bmp3.begin(0, &Wire1) /*0: low power, 1: normal, 2: high res, 3: ultra high*/) { setErr(BMP180_2_FAIL); }
  delay(10);// This delay is needed because it seems Wire1 is used for a bit after trying to connect to bmp3
  if (!adxl.begin()) { setErr(ADXL375_FAIL); }
  if (!ina219.begin()) { setErr(INA219_FAIL); }
  //adxl.setRange(ADXL345_RANGE_16_G);

  logfile = SD.open(checkFile(), FILE_WRITE);  //Opens new file with highest index
  long epochTime = epoch();
  //Print epoch bytes to logfile
  logfile.print((char)((epochTime >> 24) & 0xFF));
  logfile.print((char)((epochTime >> 16) & 0xFF));
  logfile.print((char)((epochTime >> 8) & 0xFF));
  logfile.println((char)(epochTime & 0xFF));
  logfile.println("TIME(us), Data ID, Data");  // write header at top of log file
  logfile.println("Data IDs, 0: GPS, 1: BMP280, 2: BMP180 1&2, 3: MPU6050, 4: LSM9DS1, 5: ADXL375, 6: AS5600 1&2");  // write header at top of log file
  logfile.flush();
}
//Generate the configuration string for Factory Default Settings
uint8_t setDefaults[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2F, 0xAE };

//10Hz Max data rate for SAM-M8Q
//byte setDataRate[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96 };
//setDataRate[6] = 0x64;
//setDataRate[12] = 0x7A;
//setDataRate[13] = 0x12;
uint8_t setDataRate[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12 };

//Faster 38400 Baud Rate for the higher update rates
uint8_t setBaudRate[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF, 0x70 };

//Generate the configuration string for Navigation Mode
uint8_t setNav[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F };

//Generate the configuration string for NMEA messages
uint8_t setGLL[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
uint8_t setGSA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31 };
uint8_t setVTG[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46 };
uint8_t setGGA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23 };
uint8_t set4_1[] = { 0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57 };

//Generate the configuration string for interference resistance settings
uint8_t setJam[] = { 0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45 };

//For M8 series gps, just add Galileo, from https://portal.u-blox.com/s/question/0D52p00008HKEEYCA5/ublox-gps-galileo-enabling-for-ubx-m8
uint8_t setSat[] = { 0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x82, 0x56 };

int dynamicStartStep = 0;
void dynamicStart() {  //Alleon Oxales, performs startup sequence steps every 100ms to prevent blocking of sensors in case of in-flight start.
  if ((micros() - DYN_START_LAST > DYN_START_RATE)) {
    Serial.println("DYNAMIC_START");
    switch(dynamicStartStep){
      case 0:
        if (!bmp1.begin()) { setErr(BMP280_FAIL); }else{clearErr(BMP280_FAIL);}
        if (!bmp2.begin(0, &Wire2)) { setErr(BMP180_1_FAIL); }else{clearErr(BMP180_1_FAIL);}
        if (!bmp3.begin(0, &Wire1)) { setErr(BMP180_2_FAIL); }else{clearErr(BMP180_2_FAIL);}
        break;
      case 1:
        gpsSerial.begin(38400);
        sendUBX(setDefaults, sizeof(setDefaults));
        gpsChecksum(setGSV_ON, sizeof(setGSV_ON));
        break;
      case 2:
        gpsSerial.begin(9600);
        sendUBX(setDefaults, sizeof(setDefaults));
        break;
      case 3:
        sendUBX(setGSA, sizeof(setGSA));
        break;
      case 4:
        sendUBX(setNav, sizeof(setNav));
        break;
      case 5:
        sendUBX(setDataRate, sizeof(setDataRate));
        break;
      case 6:
        sendUBX(set4_1, sizeof(set4_1));
        break;
      case 7:
        sendUBX(setSat, sizeof(setSat));
        break;
      case 8:
        sendUBX(setVTG, sizeof(setVTG));
        break;
      case 9:
        sendUBX(setJam, sizeof(setJam));
        break;
      case 10:
        sendUBX(setGLL, sizeof(setGLL));
        break;
      case 11:
        sendUBX(setGGA, sizeof(setGGA));
        break;
      case 12:
        sendUBX(setBaudRate, sizeof(setBaudRate));
        gpsSerial.end();
        gpsSerial.flush();
        gpsSerial.begin(38400);
        clearErr(SAMM8Q_FAIL);
        break;
      case 13: 
        if (!mpu.begin(104, &Wire1)) { setErr(MPU6050_FAIL); } 
        else{
          setupMPU6050();
          clearErr(MPU6050_FAIL);
        }
        break;
      case 14:
        if (!lsm.begin()) { setErr(LSM9SD1_FAIL); } 
        else{
          setupLSM();
          clearErr(LSM9SD1_FAIL);
        }
        break;
      case 15:
        if (!adxl.begin()) { setErr(ADXL375_FAIL); }
        else{
          clearErr(ADXL375_FAIL);
        }
        break;
      case 16:
        if(!ina219.begin()) { setErr(INA219_FAIL);}
        else{
          clearErr(INA219_FAIL);
        }
      case 17:
        logfile = SD.open(checkFile(), FILE_WRITE);  //Opens new file with highest index
        long epochTime = epoch();
        //Print epoch bytes to logfile
        logfile.print((char)((epochTime >> 24) & 0xFF));
        logfile.print((char)((epochTime >> 16) & 0xFF));
        logfile.print((char)((epochTime >> 8) & 0xFF));
        logfile.println((char)(epochTime & 0xFF));
        logfile.println("TIME(us), Data ID, Data");  // write header at top of log file
        logfile.println("Data IDs, 0: GPS, 1: BMP280, 2: BMP180 1&2, 3: MPU6050, 4: LSM9DS1, 5: ADXL375, 6: AS5600 1&2");  // write header at top of log file
        logfile.flush();
        clearErr(MAIN_PWR_FAULT);
    }
    DYN_START_LAST = micros();
    dynamicStartStep++;
  }
}

//==========GPS CODE==========//Based on SparkyVT https://github.com/SparkyVT/HPR-Rocket-Flight-Computer/blob/V4_7_0/Main%20Code/UBLOX_GNSS_Config.ino
bool initSAM_M8Q() {
  bool gpsReady = 1;
  
  gpsSerial.begin(38400);
  Serial.print("Set Defaults @ 38400 Baud... ");
  if (gpsSet(setDefaults, sizeof(setDefaults)) == 3) { Serial.println("Restore Defaults Failed!"); };
  gpsSerial.begin(9600);
  Serial.print("Set Defaults @ 9600 Baud... ");
  if (gpsSet(setDefaults, sizeof(setDefaults)) == 3) { Serial.println("Restore Defaults Failed!"); };

  Serial.print("Deactivating GSA Messages... ");
  if (gpsSet(setGSA, sizeof(setGSA)) == 3) {
    gpsReady = 0;
    Serial.println("NMEA GSA Message Deactivation Failed!");
  }
  Serial.print("Setting Nav Mode... ");
  if (gpsSet(setNav, sizeof(setNav)) == 3) {
    gpsReady = 0;
    Serial.println("Nav mode Failed!");
  }
  /*Serial.print("Deactivating GSV Messages... ");
  if (gpsSet(setGSV, sizeof(setGSV)) == 3) {
    gpsReady = 0;
    Serial.println("NMEA GSV Message Deactivation Failed!");
  }Moved to command stream*/
  Serial.print("Set 10Hz data rate... ");
  if (gpsSet(setDataRate, sizeof(setDataRate)) == 3) {
    gpsReady = 0;
    Serial.println("Set 10Hz data rate Failed!");
  }
  Serial.print("Activating NMEA 4.1 Messages... ");
  if (gpsSet(set4_1, sizeof(set4_1)) == 3) {
    gpsReady = 0;
    Serial.println("NMEA 4,1 Activation Failed!");
  }
  Serial.print("Set Satellites... ");
  if (gpsSet(setSat, sizeof(setSat)) == 3) {
    gpsReady = 0;
    Serial.println("Satellite Setting Failed!");
  }
  Serial.print("Deactivating VTG Messages... ");
  if (gpsSet(setVTG, sizeof(setVTG)) == 3) {
    gpsReady = 0;
    Serial.println("NMEA VTG Message Deactivation Failed!");
  }
  Serial.print("Setting Interference Threshols... ");
  if (gpsSet(setJam, sizeof(setJam)) == 3) {
    gpsReady = 0;
    Serial.println("Interference Settings Failed!");
  }
  Serial.print("Deactivating GLL Messages... ");
  if (gpsSet(setGLL, sizeof(setGLL)) == 3) {
    gpsReady = 0;
    Serial.println("NMEA GLL Message Deactivation Failed!");
  }
  Serial.print("Deactivating GGA Messages... ");
  if (gpsSet(setGGA, sizeof(setGGA)) == 3) {
    gpsReady = 0;
    Serial.println("NMEA GGA Message Deactivation Failed!");
  }

  //Increase Baud-Rate on M8Q for faster GPS updates
  //int setSucess = 0;
  Serial.print("Setting Ublox Baud Rate 38400... ");
  sendUBX(setBaudRate, sizeof(setBaudRate));
  //setSucess += getUBX_ACK(&setBaudRate[2]);
  //if (setSucess == 3 ){Serial.println("Ublox Baud Rate 38400 Failed!");}
  gpsSerial.end();
  gpsSerial.flush();
  gpsSerial.begin(38400);
  return gpsReady;
}  //end ConfigGPS
int gpsSet(uint8_t* msg, uint8_t size) {
  int gpsSetSuccess = 0;
  while (gpsSetSuccess < 3) {
    sendUBX(msg, size);
    gpsSetSuccess += getUBX_ACK(&msg[2]);
  }
  return gpsSetSuccess;
}  //end gpsSet
void gpsChecksum(uint8_t* checksumPayload, uint8_t payloadSize) {
  uint8_t CK_A = 0, CK_B = 0;
  for (int i = 2; i < payloadSize; i++) {
    if(i == payloadSize-2){
      *(checksumPayload+i) = CK_A;
      *(checksumPayload+i+1) = CK_B;
      break;
    }else{
      CK_A = CK_A + *(checksumPayload+i);
      CK_B = CK_B + CK_A;
    }
  }
}  //end gpsChecksum
void sendUBX(uint8_t* UBXmsg, uint8_t msgLength) {
  for (int i = 0; i < msgLength; i++) {
    gpsSerial.write(UBXmsg[i]);
    gpsSerial.flush();
  }
  gpsSerial.println();
  gpsSerial.flush();
}  //end sendUBX
uint8_t getUBX_ACK(uint8_t* msgID) {
  uint8_t CK_A = 0, CK_B = 0;
  uint8_t incoming_char;
  unsigned long ackWait = millis();
  uint8_t ackPacket[10] = { 0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int i = 0;
  while (1) {
    if (gpsSerial.available()) {
      incoming_char = gpsSerial.read();
      if (incoming_char == ackPacket[i]) {
        i++;
      } else if (i > 2) {
        ackPacket[i] = incoming_char;
        i++;
      }
    }
    if (i > 9) break;
    if ((millis() - ackWait) > 500) {
      Serial.println("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      Serial.println("NAK Received");
      return 1;
    }
  }

  for (i = 2; i < 8; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    Serial.println("Success!");
    Serial.print("ACK Received! ");
    //printHex(ackPacket, sizeof(ackPacket));
    return 10;
  } else {
    Serial.print("ACK Checksum Failure: ");
    //printHex(ackPacket, sizeof(ackPacket));
    //delay(1000); Removed to reduce startup time
    return 1;
  }
}  //end getACK
void readGPS(){
  static char gps_msg[200] = {};
  static int gps_msg_index = 0;
  if (gpsSerial.available() > 0) {  //Read gps messages
    char incomingByte = gpsSerial.read();

    //Print UBX messages
    /*if(record_bool){
      Serial.print(record_count-2);
      Serial.print(", ");
      Serial.print(incomingByte, HEX);
      Serial.print(", ");
      for(int i = 7; i >= 0; i--){
        Serial.print((incomingByte >> i) & 1);
      }Serial.println();
    }
    record_count+=record_bool;
    if(incomingByte == 0x62 && last_gps_byte == 0xB5){
      record_bool = 1;
    }
    if(incomingByte == '$'){
      record_bool = 0;
      record_count = 0;
    }
    last_gps_byte = incomingByte;*/
    gps_msg[gps_msg_index] = incomingByte;
    Serial.print(incomingByte);
    gps_msg_index++;
    if (incomingByte == '\n') {
      parseGPS(gps_msg, gps_msg_index);
      gps_msg_index = 0;
    }
    if(gps_msg_index > 190){gps_msg_index = 0;}//Prevent buffer overflow
  }
}  //end readGPS
void parseGPS(char* msg, uint8_t size) {  //Alleon Oxales
  uint msg_index = 0;
  uint item_index = 0;
  uint item_length = 0;
  static char item[20] = {};
  if (*msg == '$' && *(msg + 1) == 'G' && *(msg + 2) == 'N') {  //Looks for start of message with "$GN"
    msg_index += 3;
    if (*(msg + 3) == 'R' && *(msg + 4) == 'M' && *(msg + 5) == 'C') {  //Checks RMC type
      msg_index += 4;                                                   //Skips the first comma
      while (msg_index < size) {
        if (*(msg + msg_index) == ',') {
          switch (item_index) {
            case 0: break;                                   //UNIVERSAL TIME
            case 1: break;                                   //NAVIGATIONAL STATUS
            case 2: parseLatLong(item, item_length); break;  //LATITUDE
            case 4: parseLatLong(item, item_length); break;  //LONGNITUDE
            case 6: break;                                   //GROUND SPEED
            case 7: break;                                   //COURSE
            case 8: break;                                   //DATE
          }
          if (debug) {
            for (uint i = 0; i < item_length; i++) {
              Serial.print(item[i]);
            }
            Serial.print('-');
          }
          item_index++;
          item_length = 0;
        } else {
          item[item_length] = *(msg + msg_index);
          item_length++;
        }
        msg_index++;
      }
    }
  }
  msg_index = 0;
  if (*(msg + 3) == 'G' && *(msg + 4) == 'S' && *(msg + 5) == 'V') {  //Checks GSV type pg. 156 of M8 Reciever Desc.
    uint8_t sat_count = 0;
    msg_index += 7;                                                   //Skips the first comma
    while (msg_index < size) {
      if (*(msg + msg_index) == ',') {
        switch (item_index) {
          case 0: break;   //numMsg
          case 1: break;   //msgNum
          case 2:{ //numSV
            uint exp = 1;
            for(uint i = 0; i < item_length; i++){
              sat_count += (item[item_length-i-1] - '0')*exp;
              exp *= 10;
            }
            break;
          }   
          case 4: break;   //satID
          case 6: break;   //elevation
          case 7: break;   //azimuth
          case 8: break;   //signal strength
          case 9: break;   //nmea signal ID
        }
        item_index++;
        item_length = 0;
      } else {
        item[item_length] = *(msg + msg_index);
        item_length++;
      }
      msg_index++;
    }
    switch(*(msg + 2)){
      case 'P':
        gps_sats = sat_count;
        break;
      case 'L':
        glonas_sats = sat_count;
        break;
      case 'A':
        gallileo_sats = sat_count;
        break;
    }
  }
  if (debug) { Serial.print("||||"); }
}  //end parseGPS
void parseLatLong(char* value, uint8_t size) {//converts text lat, long to float, Alleon Oxales
  float power = 10000;
  float sum = 0;
  if(size == 10){power = 1000;} //Change power of ten to ten if size is for lattitude, otherwise keep it at 100
  if(size >= 10){
    for(int i = 0; i < size; i++){
      if(*(value + i) != '.'){
        sum = sum + power*(*(value + i) - '0');
        power = power/10;
      }
    }
  }
  if (size == 10) {//Latitude
    latitude = sum;  //Converting integer to float
    if(pos_start[1] == 0){pos_start[1] = sum;}//Set starting latitude
    position[1] = sum;
    logfile.print(micros());
    logfile.print("|0|");
    logfile.print(String(latitude,10));
    logfile.print('|');
  }
  if (size == 11) {//Longitude
    longitude = sum;  //Converting integer to float
    if(pos_start[0] == 0){pos_start[0] = sum;}//Set starting latitude
    position[0] = sum;
    logfile.println(String(longitude,10));
  }
}  //end parseLatLong
float gps_to_xy(float coord){//Converts gps coordinates to meters north and east, Alleon Oxales WIP
  float result = 111319.490793*(std::fmod(coord,100) + std::remainder(coord,100)/60); // 111319.490793 = 2*PI*Radius_Earth/360
  return result;
}
//============================

//==========RADIO CODE==========Alleon Oxales
void readRFD() {
  if (rfSerial.available() > 0) {         //Ping Data From Ground Station
    char incomingByte = rfSerial.read();  //Do not make this static
    rfSerial.print(incomingByte);
    if (debug) { Serial.print(incomingByte); }
    commands(incomingByte);
  }
}
void sendRFD() {
  if ((micros() - RFD_LAST > RFD_RATE)) {
    char* massage = readyPacket();
    rfSerial.flush();
    for (int i = 0; i < charArrayLegnth; i++) {
      rfSerial.print(*(massage + i));  //Send telemetry
    }
    RFD_LAST = micros();
    //printRTC();
    //printErr();
  }
}
void readRFM() {
  if (!errorCodes[RFM9X_FAIL] && rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      Serial.println(buf[0]);
      commands(buf[0]);
    }
  }
}
void sendRFM() {
  if ((micros() - RFM_LAST > RFM_RATE) && !errorCodes[RFM9X_FAIL]) {
    uint8_t* massage = reinterpret_cast<uint8_t*>(readyPacket());
    //itoa(packetnum++, radiopacket+13, 10);
    //Serial.print("Sending "); Serial.println(radiopacket);
    //radiopacket[19] = 0;

    //Serial.println("Sending...");
    //delay(10);
    rf95.send(massage, charArrayLegnth);

    //Serial.println("Waiting for packet to complete...");
    //delay(10);
    //rf95.waitPacketSent();
    RFM_LAST = micros();
  }
}
char* readyPacket() {  //Leiana Mendoza -      
  //Combines telemetry into bit array then convert to char array
  int bitArray[bitArrayLength] = {};
  static char charArray[charArrayLegnth] = {};  //+1 to include checksum byte, static so that the mem alloc is retained throughout the program
  int bitIndex = 0;
  for (int i = 0; i < 10; i++) {
    int dataLength = bitLengthList[i];
    if (i == 1) {  //Deal with ident
      bitArray[bitIndex] = sender_indent;
      bitIndex++;
    } else if ((i == 0) || (i > 1 && i < 8)) {  //Deal with floats
      float datum = 0;
      switch (i) {
        case 0: datum = time_since_launch; break;
        case 2: datum = BMP280_ALT; break;
        case 3: datum = position[1]; break;
        case 4: datum = position[0]; break;
        case 5: datum = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]); break;
        case 6: datum = 255; break;
        case 7: datum = 255; break;
      }
      int* bitsPtr;
      if(i == 3 || i == 4){//Handle Lat and Lon
        bitsPtr = float_to_binary(datum);
      }else{
        bitsPtr = dec_to_binary(datum, dataLength);
      }
      for (int x = 0; x < dataLength; x++) {
        //Serial.print(*(bitsPtr+dataLength-1-x));
        bitArray[bitIndex] = *(bitsPtr + dataLength - 1 - x);
        bitIndex++;
      }
    } else if (i == 8) {  //Deal with error codes
      for (int x = 0; x < dataLength; x++) {
        bitArray[bitIndex] = errorCodes[x];
        bitIndex++;
      }
    } else if (i == 9) {  //Deal with  bits
      for (int x = 0; x < dataLength; x++) {
        bitArray[bitIndex] = my_event_arr[x];
        bitIndex++;
      }
    }
    charArray[charArrayLegnth - 3] = radioChecksum(bitArray, bitArrayLength);
    charArray[charArrayLegnth - 2] = 'R';
    charArray[charArrayLegnth - 1] = 'L';
  }
  int charIndex = 0;
  for (int i = 0; i < bitArrayLength; i += 8) {
    char result = 0;
    for (int j = 0; j <= 7; j++){
      result |= (bitArray[i + j] << (7-j));
    }
    charArray[charIndex] = result;
    charIndex++;
  }
  return charArray;
}  //end readyPacket
uint8_t radioChecksum(int* radioMSG, uint8_t msgLength) {//fletcher 8 checksum algorithm
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (size_t i = 0; i < msgLength; i++) {
      sum1 = (sum1 + radioMSG[i]) % 15;
      sum2 = (sum2 + sum1) % 15;
  }
  return (sum2 << 4) | sum1;
}  //end radioChecksum
int* dec_to_binary(float my_dec, int my_bit) {  //Ellie McGhee, Returns elements in reverse order
  int my_dec_int = static_cast<int>(round(my_dec));
  int my_rem;
  static int my_arr[32];
  if (my_bit > 32) {
    Serial.println("Array requested too large");
    setErr(PRGM_ERR);
  }
  for (int i = my_bit - 1; i >= 0; i--) {
    int my_exp = pow(2.0, i);
    my_rem = my_dec_int % my_exp;
    if (my_dec_int >= my_exp) {
      my_arr[i] = 1;
    } else if (my_dec_int < my_exp) {
      my_arr[i] = 0;
    }
    my_dec_int = my_rem;
  }
  return my_arr;
}  //end dec_to_binary
int* float_to_binary(float num) {//Aleon Oxales
  static int bit_array[32] = {};
  uint32_t *int_repr = reinterpret_cast<uint32_t *>(&num);

  // Extract individual bits
  for (int i = 31; i >= 0; i--) {
    bit_array[i] = (*int_repr >> i) & 1;
  }
  return bit_array; 
}
//Binary to decimal (Elizabeth McGhee)
float binary_to_dec(int my_bit_size, int* my_arr) {
  float my_sum = 0;
  int my_index = 0;
  for (int i = my_bit_size - 1; i >= 0; i--) {
    my_sum = pow(2, my_index) * *(my_arr + i) + my_sum;
    my_index++;
  }
  return my_sum;
}  //end binary to decimal
int* uint_to_binary(char character) { //Leiana Mendoza
  static int result[8];
  for (int i = 0; i <= 7; i++) {
    result[i] = (character & (1 << (7-i))) > 0;
  }
  return result;
}  //end uint_to_binary
//==============================

// Event Detection ============================================ Elizabeth McGhee WIP
void event_detection() {
   
  float burn = 684.276;  //We don't know this yet
  float g = 9.81;
  double speed = velocity_magnitude();
  bool liftoff = ((LSM_AZ > 2*g && ADXL345_AZ > 2*g && MPU_AZ > 2*g) && (BMP280_ALT > 50 && BMP180_1_ALT > 50) && (speed > 20));
  bool burnout = ((LSM_AZ < 1*g && ADXL345_AZ < 1*g && MPU_AZ < 1*g) && (BMP280_ALT > burn && BMP180_1_ALT > burn));
  bool apogee = ((LSM_AX < 0 && LSM_AY < 0 && LSM_AZ < 0) && (MPU_GX < 0 && MPU_GY < 0 && MPU_GZ < 0));
  bool drogue_deploy = ((speed < 17.3736) && (apogee==1));
  bool main_deploy = ((BMP280_ALT < 457.2 && BMP180_1_ALT < 457.2) && (speed < 7.9248));
  bool landed = (speed < 5 && (BMP280_ALT < 50 && BMP180_1_ALT < 50));

  // The index is in the following ascending order: liftoff, burnout, apogee, drogue deploy, main deplot, landed
  if (liftoff) {
    my_event_arr[0] = 1;
  } else {
    my_event_arr[0] = 0;
  }
  if (burnout) {
    my_event_arr[1] = 1;
  } else {
    my_event_arr[1] = 0;
  }
  if (apogee) {
    my_event_arr[2] = 1;
  } else {
    my_event_arr[2] = 0;
  }
  if (drogue_deploy){
    my_event_arr[3] = 1;
  } else {
    my_event_arr[3] = 0;
  }
  if (main_deploy){
    my_event_arr[4] = 1;
  } else {
    my_event_arr[4] = 0;
  }
  if (landed) {
    my_event_arr[5] = 1;
  } else {
    my_event_arr[5] = 0;
  }
}
bool detect_good_shutdown() {  //Alleon Oxales
  bool eeprom_check = 1;
  for (int i = 0; i < 3; i++) {  //First looks for shutdown flag in EEPROM
    int value = EEPROM.read(i);
    if (value == 0xff) {
      EEPROM.write(i, 0);
    } else {
      eeprom_check = 0;
      break;
    }
  }

  if (!eeprom_check) {  //If eeprom check fails, try SD card check
    if (errorCodes[SD_FAIL]) {
      return false;
    }
    long epochTime = 0;
    logfile = SD.open(checkFile(1), FILE_READ);
    for (int i = 3; i >= 0; i--) {
      if (logfile.available()) {
        epochTime += logfile.read() * (1 << (i * 8));  // Read one character and add to epoch
      } else {
        return true;  //If for example, there are no log files yet
      }
    }
    logfile.close();
    if (epoch() - epochTime > 3600) {  //Check if log file creation was >1hr ago
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}  //end detect_good_shutdown
//========================================

//=========SENSOR CODE==========
float bmpAplyCalibrat(int sensor_id, float val){
  switch (sensor_id){
    case 0:
      return val+1592.34+(val-99830)*0.49;
    case 1:
      return val-14377+(val-115800)*(-0.14);
    case 2:
      return val-23577+(val-125000)*(-0.14);
  }
  return 0;
}

float lat_to_meters(float val) {
  return (val * 111.32)/1000;
}

float lon_to_meters(float val){
  float x = 40075 * cos( latitude ) / 360;
  return x/1000;
}

float velocity_magnitude(){
  float T = 280;
  float gamma = 1.4;
  float R = 287;
  float stag_press = BMP180_1_PRESS;
  float static_press = BMP180_2_PRESS;

  float velocity = sqrt(((2*gamma*R*T)/(gamma - 1))*(pow((stag_press/static_press),(gamma - 1)/1)) - 1);
  return velocity;
}
void get_down_tonight(float* accel_dir, float* mag_dir){
  //Step 1: get the quaternion needed to rotate our body up vector to the measured world up vector from gravity.
  static float up[3] = {0,0,1};
  static float north[3] = {1,0,0};
  float cross_result[3];
  float* cross_1 = cross_product(up, accel_dir, cross_result);
  float cross_1_mag = vec_mag(cross_1);
  float rot_vec[3] = {cross_1[0]/cross_1_mag,cross_1[1]/cross_1_mag,cross_1[2]/cross_1_mag};
  float theta_half = acos(dot_product(up,accel_dir)/vec_mag(accel_dir));
  float quat1[4] = {cos(theta_half),
    rot_vec[0]*sin(theta_half),
    rot_vec[1]*sin(theta_half),
    rot_vec[2]*sin(theta_half)};
  //Get magnetometer projection along XY plane
  float proj_len = dot_product(mag_dir, accel_dir)/dot_product(accel_dir,accel_dir);
  float mag_proj[3] = {mag_dir[0] - accel_dir[0]*proj_len,
    mag_dir[1] - accel_dir[1]*proj_len,
    mag_dir[2] - accel_dir[2]*proj_len
  };
  //Step 2: rotate the magnetometer vector by the quaternion
  float M[4] = {0,mag_proj[0],mag_proj[1],mag_proj[2]};
  float my_rot[4];
  float* M_rot = quat_rot(quat1, M, my_rot);
  //Step 3: get the quaternion needed to rotate body_x  (north) to measured north
  float theta_2_half = acos(dot_product(north,M_rot)/vec_mag(M_rot));
  float quat2[4] = {cos(theta_2_half),0,0,sin(theta_2_half)};
  //Step 4: get final quaternion 
  float quat[4];
  float* result = quat_mul(quat1, quat2, quat);
  quat_start[0] = *(result); quat_start[1] = *(result+1); quat_start[2] = *(result+2); quat_start[3] = *(result+3);
}
// vector<double> quaternion_to_speed(double a, double b, double c, double d){
//   vector<double> q = {a, b, c, d};
//   vector<double> direction = {0, 0, 1};
//   vector<double> q_star = {a, -b, -c, -d};
//   vector<double> new_q = {q[1]*direction[1]*q_star[1], q[2]*direction[2]*q_star[2], q[3]*direction[3]*q_star[3]};

//   return new_q;
//   }

float alt_from_pres(float val){
  return 44330.0*(1-pow(press_start/val,0.1903)); //Based on Adafruit_BMP_085_Unified
}
void TCA9548A(uint8_t bus) {     // Select I2C Bus On I2C Multiplexer (if used)
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send uint8_t to select bus
  Wire.endTransmission();
  Serial.print(bus);
}  //end TCA9548A
void setupLSM() {  //Ryan Santiago
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_952HZ); //Options 2/4/8/16G 10/119/476/952Hz

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS); //Options 4/8/12/16

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); //Options 245/500/2000
}  //end setupLSM
void setupMPU6050(){
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);//Options: 2/4/8/16
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);//Options 250/500/1000/2000
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);//Options 260/184/94/44/21/10/5
  mpu.setSampleRateDivisor(0); // Divides Gyro output rate by (Sample divisor + 1) to get sample rate
}
void get_mag_accel_samps(float* vals, bool reset = 0, bool mode = 0 /*0: accel samp, 1: mag samp*/){//This function was moved up here because it needs to be in-context of loop() since it takes in input
  static int accel_count = 0;
  static int mag_count = 0;
  static float accel_avg[3] = {0,0,9.8};
  static float mag_avg[3] = {0,0,0};
  static int start_time = 0;
  if(reset){
    accel_count = 0;
    mag_count = 0;
    accel_avg[0] = 0;accel_avg[1] = 0; accel_avg[2] = 9.8;
    mag_avg[0] = 0;mag_avg[1] = 0; mag_avg[2] = 0;
    start_time = millis();
  }else if(millis()-start_time < 1000 && !mode){//Only sample for 1000ms
    accel_avg[0] = (accel_avg[0]*accel_count + vals[0])/(accel_count+1);
    accel_avg[1] = (accel_avg[1]*accel_count + vals[1])/(accel_count+1);
    accel_avg[2] = (accel_avg[2]*accel_count + vals[2])/(accel_count+1);
    accel_count++;
  }else if(millis()-start_time < 1000 && mode){//Only sample for 1000ms
    mag_avg[0] = (mag_avg[0]*mag_count + vals[0])/(mag_count+1);
    mag_avg[1] = (mag_avg[1]*mag_count + vals[1])/(mag_count+1);
    mag_avg[2] = (mag_avg[2]*mag_count + vals[2])/(mag_count+1);
    mag_count++;
  }else{
    accel_start[0] = accel_avg[0];
    accel_start[0] = accel_avg[1];
    accel_start[2] = accel_avg[2];
    mag_start[0] = mag_avg[0];
    mag_start[0] = mag_avg[1];
    mag_start[2] = mag_avg[2];
    get_down_tonight(accel_start, mag_start);
  }
}
void readMPU6050(){    //Elizabeth McGhee
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //Callibrated
  MPU_AX = a.acceleration.x + 0.6163405527;
  MPU_AY = a.acceleration.y - 0.02601792382;
  MPU_AZ = a.acceleration.z - 0.7746213592;
  MPU_GX = g.gyro.x + 0.05;
  MPU_GY = g.gyro.y;
  MPU_GZ = g.gyro.z - 0.02;

  if(!errorCodes[SD_FAIL]){
    logfile.print(micros());
    logfile.print("|3|");
    logfile.print(MPU_AX);logfile.print('|');
    logfile.print(MPU_AY);logfile.print('|');
    logfile.print(MPU_AZ);logfile.print('|');
    logfile.print(MPU_GX);logfile.print('|');
    logfile.print(MPU_GY);logfile.print('|');
    logfile.println(MPU_GZ);
  }
}
void readLSM() {  //Alleon Oxales
  //Read sensor data
  lsm.read();
  lsm.getEvent(&LSM_acc, &LSM_mag, &LSM_gyro, &LSM_temp);
  //Callibrated
  LSM_AX = -LSM_acc.acceleration.y + 0.4984510829;
  LSM_AY = -LSM_acc.acceleration.x - 0.5102867812;
  LSM_AZ = LSM_acc.acceleration.z + 0.2804271845;

  float temp[3] = {LSM_AX,LSM_AY,LSM_AZ};
  get_mag_accel_samps(temp, 0, 0);//Start recording values to get gravity direction at pad;

  LSM_GX = -LSM_gyro.gyro.y - 0.000337512054;
  LSM_GY = -LSM_gyro.gyro.x + 0.02429604629;
  LSM_GZ = LSM_gyro.gyro.z + 0.02113307618;
  LSM_MX = -LSM_mag.magnetic.y;
  LSM_MY = -LSM_mag.magnetic.x;
  LSM_MZ = LSM_mag.magnetic.z;

  temp[0] = LSM_AX;
  temp[1] = LSM_AY;
  temp[2] = LSM_AZ;
  get_mag_accel_samps(temp, 0, 1);
  
  if(!errorCodes[SD_FAIL]){
    logfile.print(micros());
    logfile.print("|4|");
    logfile.print(LSM_AX);logfile.print('|');
    logfile.print(LSM_AY);logfile.print('|');
    logfile.print(LSM_AZ);logfile.print('|');
    logfile.print(LSM_GX);logfile.print('|');
    logfile.print(LSM_GY);logfile.print('|');
    logfile.print(LSM_GZ);logfile.print('|');
    logfile.print(LSM_MX);logfile.print('|');
    logfile.print(LSM_MY);logfile.print('|');
    logfile.println(LSM_MZ);
  }
  if (debug) {
    Serial.print("ACC:");
    Serial.print(LSM_acc.acceleration.x, 2);
    Serial.print(", ");
    Serial.print(LSM_acc.acceleration.y, 2);
    Serial.print(", ");
    Serial.print(LSM_acc.acceleration.z, 2);
    Serial.print("   ");

    Serial.print("ROT:");
    Serial.print(LSM_gyro.gyro.x, 2);
    Serial.print(", ");
    Serial.print(LSM_gyro.gyro.y, 2);
    Serial.print(", ");
    Serial.print(LSM_gyro.gyro.z, 2);
    Serial.print("   ");

    Serial.print("MAG:");
    Serial.print(LSM_mag.magnetic.x, 2);
    Serial.print(", ");
    Serial.print(LSM_mag.magnetic.y, 2);
    Serial.print(", ");
    Serial.print(LSM_mag.magnetic.z, 2);
    Serial.print("   ");
    Serial.print("||||");
  }
}
void readADXL() {
  sensors_event_t event;
  adxl.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) with callibration */
  ADXL345_AX = -event.acceleration.x + 7.497139656;
  ADXL345_AY = -event.acceleration.y + 7.772019417;
  ADXL345_AZ = event.acceleration.z - 1.129876027;

  if(!errorCodes[SD_FAIL]){
    logfile.print(micros());
    logfile.print("|5|");
    logfile.print(ADXL345_AX);logfile.print('|');
    logfile.print(ADXL345_AY);logfile.print('|');
    logfile.println(ADXL345_AZ);
  }
}  //end readADXL
int readRawAngle(TwoWire* wire) {  //Ryan Santiago
  wire->beginTransmission(AS5600_ADDR);
  wire->write(RAW_ANGLE_REG);  // Set the register to read raw angle
  wire->endTransmission();

  // Request 2 bytes from AS5600
  wire->requestFrom(AS5600_ADDR, 2);
  if (wire->available() == 2) {
    int highByte = wire->read();
    int lowByte = wire->read();

    // Combine high and low uint8_t to form a 12-bit result
    int angle = (highByte << 8) | lowByte;

    // Return the angle
    return angle;
  } else {
    // Return -1 if reading fails
    return -1;
  }
}  //end readRawAngle
void mag_apply_calibrat(float* vals){//Based on matlab callibration values
  static const float A[3][3] = {
    {0,0,0},
    {0,0,0},
    {0,0,0}
  };
  static const float B[3] = {0,0,0};
  for(int i = 0; i < 3; i++){vals[i]-=B[i];}//Do offset before matrix multiplication
  // Multiply: temp * A
  float result[3] = {0,0,0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        result[i] += vals[j] * A[j][i];
    }
  }
  vals = result;
}
//==============================

void setErr(int errorCode) {
  errorCodes[errorCode] = 1;
}  //end setErr
void clearErr(int errorCode) {
  errorCodes[errorCode] = 0;
}  //end clearErr
void commands(char command) {  //Alleon Oxales
  if (shutdownCheck[2] == 0 && millis() - time_last_command > 1000) {
    memset(shutdownCheck, 0, sizeof(shutdownCheck));//Reset shutdown checks after 1s timeout
  }
  switch (command) {  //Check for commands
    case 0xff:
      for (int i = 0; i < 3; i++) {  //Set the latest shutdownCheck
        if (shutdownCheck[i] == 0 && (my_event_arr[5] || !my_event_arr[0])) {//Only allow shutdown command to run during landing
          Serial.println('F');
          shutdownCheck[i] = 0xFF;
          break;
        }
      }
      if (shutdownCheck[0] == 0xff && shutdownCheck[1] == 0xff && shutdownCheck[2] == 0xff) {  //Write shutdown flag to EEPROM
        EEPROM.write(0, 0xff);
        EEPROM.write(1, 0xff);
        EEPROM.write(2, 0xff);
        logfile.close();
        if (debug) { Serial.println("Shutdown!"); }
      }
      break;
    case 0x01:
      lowPower = 1;
      break;
    case 0x02:
      if(!my_event_arr[0]){//Prevent low power during flight
        lowPower = 0;
      }
      break;
    case 0x03:
      videoPower = 1;
      break;
    case 0x04:
      if(!my_event_arr[0]){//Prevent video off during flight
        videoPower = 0;
        time_vid_off = millis();
      }
      break;
    case 0x05:
      for (int i = 0; i < 3; i++) {  //Set the latest shutdownCheck
        if (restartCheck[i] == 0 && !my_event_arr[0]) { //Prevent restart during flight
          restartCheck[i] = 0xFF;
          break;
        }
      }
      if (restartCheck[0] == 0xff && restartCheck[1] == 0xff && restartCheck[2] == 0xff) {  //Write shutdown flag to EEPROM
        EEPROM.write(0, 0xff);
        EEPROM.write(1, 0xff);
        EEPROM.write(2, 0xff);
        if (debug) { Serial.println("Restart!"); }
        normalStart();
        errorCodes[MAIN_PWR_FAULT] = 0;
        memset(shutdownCheck, 0, sizeof(shutdownCheck));
        memset(restartCheck, 0, sizeof(restartCheck));
      }
      break;
    case 0x06:{
      if(pwr_gps_status_poll){
        gpsSet(setGSV_OFF, sizeof(setGSV_OFF));
        pwr_gps_status_poll = 0;
      }else if(!my_event_arr[0]){//Prevent extra status messages during flight
        gpsSet(setGSV_ON, sizeof(setGSV_ON));
        pwr_gps_status_poll = 1;
      }
      break;
    }
    case 0x07:
      if(!my_event_arr[0]){//Prevent navigation alignment during flight
        pres_start_samples(0,1);//Resets sample timer to start taking samples
        AoA_start1 = readRawAngle(&Wire);//sets zero for angle of attack sensors
        AoA_start2 = readRawAngle(&Wire1);//sets zero for angle of attack sensors
        if(!errorCodes[SD_FAIL]){
          logfile.print(micros());
          logfile.print("|AoA_Callib|");
          logfile.print(AoA_start1);
          logfile.print('|');
          logfile.println(AoA_start2);
        }
        get_mag_accel_samps(0,1);
      }
      break;
    case 0x08:
      break;
  }
  time_last_command = millis();
}  //end commands
void printErr() {
  for (int i = 0; i < 32; i++) {
    int check = i + errorCodes[i] * 32;
    switch (check) {
      case PRGM_ERR + 32: Serial.print("PRGM_ERR  "); break;
      case ALT_OUT_RANGE + 32: Serial.print("ALT_OUT_RANGE  "); break;
      case LAT_OUT_RANGE + 32: Serial.print("LAT_OUT_RANGE  "); break;
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
      case ADXL375_FAIL + 32: Serial.print("ADXL375_FAIL  "); break;
      case LSM9SD1_FAIL + 32: Serial.print("LSM9SD1_FAIL  "); break;
    }
  }
  Serial.println();
}  //end printErr
void printRTC() {
  rtc.refresh();

  Serial.print("Current Date & Time: ");
  Serial.print(rtc.year());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());

  Serial.print(" (");
  Serial.print(daysOfTheWeek[rtc.dayOfWeek() - 1]);
  Serial.print(") ");

  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.println(rtc.second());
}  //end printRTC
long epoch() {
  rtc.refresh();
  struct tm timeinfo = { 0 };  // Initialize all fields to zero

  // Set up date and time components
  timeinfo.tm_year = rtc.year() + 100;  // Years since 1900
  timeinfo.tm_mon = rtc.month() - 1;    // Month, where 0 = January
  timeinfo.tm_mday = rtc.day();         // Day of the month
  timeinfo.tm_hour = rtc.hour();
  timeinfo.tm_min = rtc.minute();
  timeinfo.tm_sec = rtc.second();

  // Convert to time_t (epoch time)
  time_t epochTime = mktime(&timeinfo);

  return static_cast<long>(epochTime);
}  // end epoch

//=========MATH CODE==========
float* cross_product(float* a, float* b, float* result){
  result[0] = a[1]*b[2]-b[1]*a[2];
  result[1] = -(a[0]*b[2]-b[0]*a[2]);
  result[2] = a[0]*b[1]-b[0]*a[1];
  return result;
}

float vec_mag(float* a){
  return sqrt(pow(a[0],2)+pow(a[1],2)+pow(a[2],2));
}
float dot_product(float* a, float* b){
  return (a[0]*b[0]) + (a[1]*b[1]) + (a[2]*b[2]);
}
float* quat_mul(float* a, float* b, float* result){
  result[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
  result[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
  result[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
  result[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);
  return result;
  };
 
float* quat_rot(float* q, float* v, float my_result[4]){
  float V[4] = {0,v[0],v[1],v[2]};
  float my_arr[4];
  float my_arr_mul[4];
  float* result = quat_mul(q,V, my_arr);
  float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
  my_result = quat_mul(result, q_conj, my_arr_mul) + 1;
  return my_result;
   //Start the second element because that's where the juicy stuff is i.e. the actual rotated vector compponents
  }
