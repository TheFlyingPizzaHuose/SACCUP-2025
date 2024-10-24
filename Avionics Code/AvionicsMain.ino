/*
Saint Louis University Rocket Propulsion Laboratory (SLURPL)

Avionics Main Code
Authors: Alleon Oxales, Ryan Santiago, Elizabeth McGhee

All pertinent information for this code is in document linked bellow.
This includes all references and credit to the authors of code that
is used in this program such as libraries.

Avionics Datasheet
https://docs.google.com/document/d/138thbxfGMeEBTT3EnloltKJFaDe_KyHiZ9rNmOWPk2o/edit
*/
#include <SoftwareSerial.h> // UART Library
#include <Wire.h> // I2C library
#include <SPI.h> // SPI library
#include <SD.h> // SD card library
#include <Adafruit_BMP085.h> // BMP 085 or 180 pressure sensor library
#include <Adafruit_BMP280.h> // BMP 280 pressure sensor library
#include <Adafruit_MPU6050.h> // MPU6050 accelerometer sensor library
#include <Adafruit_Sensor.h> // Adafruit unified sensor library
#include <Adafruit_LSM9DS1.h> // Include LSM9DS1 library
#include <cmath>
#include <iostream>
#include <EEPROM.h>

#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

#define BMP_SCL 13
#define BMP_SDO 12
#define BMP_SDA 11

//UART Serial Objects
#define rfSerial Serial1
#define gpsSerial Serial2

//SD card variables 
const int sdSelect = 10;
char* logFileName;
File logfile;

// Create the sensor objects
Adafruit_MPU6050 mpu; //Accelerometer
const int bmpSelect = 9;
Adafruit_BMP280 bmp1(&Wire); //Pitot Tube Pressure Sensor
Adafruit_BMP085 bmp2;

int gpsVersion = 2; //SAM-M8Q

// AV Modes
bool lowPower = 0;
bool lowDataTransfer = 0;
int shutdownCheck[3] = {0,0,0}; //All three elements must be >0 to activate shutdown
int time_last_command = 0;

int bitLengthList[10] = {12,//Seconds since launch
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
int events[6] = {};
int errorCodes[32] = {}; //Check document for error code list

//Error codes
int PRGM_ERR = 0,
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
    SD_FAIL = 24;

//Component cycle time micros
uint BMP280_RATE = 37500,//Ultra low: 5.5, Low: 7.5, Standard: 11.5, High: 19.5, Ultra High: 37.5
    SAMM8Q_RATE = 100000,
    MPU6050_RATE = 9,
    BMP180_RATE = 17,//Ultra low: 3, Standard: 5, High: 9, Ultra High: 17, Adv. High: 51
    AS5600_RATE = 12,
    RFD_RATE = 50000;

//Component last cycle time micros
uint BMP280_LAST = 0,
    SAMM8Q_LAST = 0,
    MPU6050_LAST = 0,
    BMP180_LAST = 0,
    AS5600_LAST = 0,
    RFD_LAST = 0;

//Time variables
uint time_launch = 0;
uint last_millis = 0;
uint last_micros = 0;
float time_since_launch = 0;//seconds

int sender_indent = 1;//0: Avionics 1: Payload drone
float altitude = 0,//meters
      latitude = 0,//meters
      longitude = 0,//meters
      velocity = 0,//meters per second
      orientationX = 0,//degrees
      orientationY = 0;//degrees

//Kinematics variables
float abs_pos[3] = {0,0,0},//Absolute position measurements
      dt_pos[3] = {0,0,0},//Derivative position measurements
      d2t_pos[3] = {0,0,0},//2nd Derivative position measurement
      abs_rot[3] = {0,0,0},//Absolute rotation measurements
      dt_rot[3] = {0,0,0},//Derivatie rotation measurements
      d2_rot[3] = {0,0,0};//2nd Derivative rotation measurements
void setup() {
  Serial.begin(57600);// Start hardware serial communication (for debugging)
  Serial.println("Initializing");
  Wire.begin();
  Wire1.begin();
  
  uint hunds = '0' - '0', tens = '9' - '0', ones = '0' - '0';//Converting string to integer
  longitude = static_cast<float>((hunds*100) + (tens*10) + ones);//Converting integer to float
  Serial.println(longitude);
  if(true || detect_good_shutdown()){
    
    rfSerial.begin(57600);//Init RFD UART
    Serial.println("Software serial initialized.");
    
    //int status = initSAM_M8Q();
    if (!initSAM_M8Q()) {setErr(SAMM8Q_FAIL);}//Init SAM_M8Q and error if fails
    if (!mpu.begin()) {setErr(MPU6050_FAIL);}//Init MPU6050 and error if fails   
    //int status = bmp1.begin(0x77);
    //int status = bmp2.begin(3, &Wire1);
    //Serial.println(status);
    if (!bmp1.begin()) {setErr(BMP280_FAIL);}//Init BMP280 and error if fails  
    if (!bmp2.begin(3, &Wire1)/*0: low power, 1: normal, 2: high res, 3: ultra high*/) {setErr(BMP180_1_FAIL);}//Init BMP280 and error if fails   
    //if (!SD.begin(sdSelect)) {setErr(SD_FAIL);}//Init SD reader and error if fails

    logFileName = checkFile(); //Looks for log files already present
    logfile = SD.open(logFileName, FILE_WRITE);
    logfile.println("micros, bmp_alt (m), bmp_pressure (Pa), bmp_temp (C), solenoid state, launch detected, apogee detected, landed, ground detection count"); // write header at top of log file
    logfile.close();

    setupSensors();
  }else{
    setErr(MAIN_PWR_FAULT);
  }
}

void loop() {
  static char incomingByte;
  if(micros() < last_micros){//Clock rollover check
    BMP280_LAST = 0;SAMM8Q_LAST = 0;MPU6050_LAST = 0;BMP180_LAST = 0;AS5600_LAST = 0;}
  if (rfSerial.available() > 0) {//Ping Data From Ground Station
    incomingByte = rfSerial.read();
    rfSerial.print(incomingByte);
    Serial.print(incomingByte);
    commands(incomingByte);
  }
  static char gps_msg[200] = {};
  static int gps_msg_index = 0;
  if (gpsSerial.available() > 0) {//Print gps messages
    char incomingByte = gpsSerial.read();
    //Serial.print(incomingByte);
    gps_msg[gps_msg_index] = incomingByte;
    gps_msg_index++;
    if(incomingByte == '\n'){
      readGPS(gps_msg, gps_msg_index);
      gps_msg_index = 0;
    }
  }

  time_since_launch = micros()/1000000 - time_launch;
  if(micros() - BMP280_LAST > BMP280_RATE){
    altitude = (bmp2.readAltitude(102500)+bmp1.readAltitude(1025))/2; /* Adjusted to local forecast! */
    BMP280_LAST = micros();
  }
  if(lowDataTransfer && (micros() - RFD_LAST > RFD_RATE)){
    char* massage = readyPacket();
    for(int i = 0; i< charArrayLegnth; i++){
      //char aChar = '0' + (i%10);
      //Serial.print(aChar);
      //Serial.print(*(massage+i));
      //int* bins = uint_to_binary(*(massage+i));
      for(int x = 0; x<8; x++){
        //Serial.print(*(bins+x));
      }
    }
    for(int i = 0; i< charArrayLegnth; i++){
      rfSerial.print(*(massage+i));//Send telemetry
    }
    //rfSerial.println();
    RFD_LAST = micros();
  }


  /* Read sensor data
  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);
  
  // Send data over RFD900 (via SoftwareSerial)
  rfSerial.print("ACCEL "); rfSerial.print(accel.acceleration.x, 2); rfSerial.print(",");
  rfSerial.print(accel.acceleration.y, 2); rfSerial.print(",");
  rfSerial.print(accel.acceleration.z, 2);

  rfSerial.print(" GYRO "); rfSerial.print(gyro.gyro.x, 2); rfSerial.print(",");
  rfSerial.print(gyro.gyro.y, 2); rfSerial.print(",");
  rfSerial.print(gyro.gyro.z, 2); rfSerial.print(" rad/s");

  rfSerial.print(" MAG "); rfSerial.print(mag.magnetic.x, 2); rfSerial.print(",");
  rfSerial.print(mag.magnetic.y, 2); rfSerial.print(",");
  rfSerial.print(mag.magnetic.z, 2); rfSerial.print(" gauss");

  rfSerial.print(" TEMP "); rfSerial.print(bmp.readTemperature()); rfSerial.print("C");
  
  rfSerial.print(" PRESS "); rfSerial.print(bmp.readPressure()); rfSerial.print("Pa");
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  rfSerial.print(" ALT = "); rfSerial.print(bmp.readAltitude()); rfSerial.print("m");
  rfSerial.println();*/

  //delay(50);
}

//==========GPS CODE==========//Based on SparkyVT https://github.com/SparkyVT/HPR-Rocket-Flight-Computer/blob/V4_7_0/Main%20Code/UBLOX_GNSS_Config.ino
bool initSAM_M8Q(){
  bool gpsReady = 1;
  //Generate the configuration string for Factory Default Settings
  byte setDefaults[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2F, 0xAE};

  gpsSerial.begin(38400);
  Serial.print("Set Defaults @ 38400 Baud... ");
  if(gpsSet(setDefaults, sizeof(setDefaults)) == 3){Serial.println("Restore Defaults Failed!");};
  gpsSerial.begin(9600);
  Serial.print("Set Defaults @ 9600 Baud... ");
  if(gpsSet(setDefaults, sizeof(setDefaults)) == 3){Serial.println("Restore Defaults Failed!");};

  //10Hz Max data rate for SAM-M8Q
  byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};
  setDataRate[6] = 0x64; setDataRate[12] = 0x7A; setDataRate[13] = 0x12;

  //Faster 38400 Baud Rate for the higher update rates
  byte setBaudRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF, 0x70};
  
  //Generate the configuration string for Navigation Mode
  byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F};

  //Generate the configuration string for NMEA messages
  byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
  byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
  byte setGGA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
  byte set4_1[] = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57};

  //Generate the configuration string for interference resistance settings
  byte setJam[] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45};
  
  //For M8 series gps, just add Galileo, from https://portal.u-blox.com/s/question/0D52p00008HKEEYCA5/ublox-gps-galileo-enabling-for-ubx-m8
  byte setSat[] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x82, 0x56};

  Serial.print("Deactivating GSA Messages... ");
  if(gpsSet(setGSA, sizeof(setGSA)) == 3){gpsReady = 0; Serial.println("NMEA GSA Message Deactivation Failed!");}
  Serial.print("Setting Nav Mode... ");
  if(gpsSet(setNav, sizeof(setNav)) == 3){gpsReady = 0; Serial.println("Nav mode Failed!");}
  Serial.print("Deactivating GSV Messages... ");
  if(gpsSet(setGSV, sizeof(setGSV)) == 3){gpsReady = 0; Serial.println("NMEA GSV Message Deactivation Failed!");}
  Serial.print("Set 10Hz data rate... ");
  if(gpsSet(setDataRate, sizeof(setDataRate)) == 3){gpsReady = 0; Serial.println("Set 10Hz data rate Failed!");}
  Serial.print("Activating NMEA 4.1 Messages... ");
  if(gpsSet(set4_1, sizeof(set4_1)) == 3){gpsReady = 0; Serial.println("NMEA 4,1 Activation Failed!");}
  Serial.print("Set Satellites... ");
  if(gpsSet(setSat, sizeof(setSat)) == 3){gpsReady = 0; Serial.println("Satellite Setting Failed!");}
  Serial.print("Deactivating VTG Messages... ");
  if(gpsSet(setVTG, sizeof(setVTG)) == 3){gpsReady = 0; Serial.println("NMEA VTG Message Deactivation Failed!");}
  Serial.print("Setting Interference Threshols... ");
  if(gpsSet(setJam, sizeof(setJam)) == 3){gpsReady = 0; Serial.println("Interference Settings Failed!");}
  Serial.print("Deactivating GLL Messages... ");
  if(gpsSet(setGLL, sizeof(setGLL)) == 3){gpsReady = 0; Serial.println("NMEA GLL Message Deactivation Failed!");}
  Serial.print("Deactivating GGA Messages... ");
  if(gpsSet(setGGA, sizeof(setGGA)) == 3){gpsReady = 0; Serial.println("NMEA GGA Message Deactivation Failed!");}
  
  //Increase Baud-Rate on M8Q for faster GPS updates
  int setSucess = 0;
  while(gpsVersion == 2 && setSucess < 3) {
    Serial.print("Setting Ublox Baud Rate 38400... ");
    sendUBX(setBaudRate, sizeof(setBaudRate));
    setSucess += getUBX_ACK(&setBaudRate[2]);}
  if (setSucess == 3 ){Serial.println("Ublox Baud Rate 38400 Failed!");}
  if(gpsVersion == 2){
    gpsSerial.end();
    gpsSerial.flush();
    gpsSerial.begin(38400);
  }
  return gpsReady;
}//end ConfigGPS
int gpsSet(byte* msg, byte size){
  int gpsSetSuccess = 0;
  while(gpsSetSuccess < 3) {
      sendUBX(msg, size);
      gpsSetSuccess += getUBX_ACK(&msg[2]);}
  return gpsSetSuccess;
}//end gpsSet
void gpsChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;}
    
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}//end gpsChecksum
void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    gpsSerial.write(UBXmsg[i]);
    gpsSerial.flush();
  }
  gpsSerial.println();
  gpsSerial.flush();
}//end sendUBX
byte getUBX_ACK(byte *msgID) {
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
    if (gpsSerial.available()) {
      incoming_char = gpsSerial.read();
      if (incoming_char == ackPacket[i]) {
        i++;}
      else if (i > 2) {
        ackPacket[i] = incoming_char;
        i++;}}
    if (i > 9) break;
    if ((millis() - ackWait) > 1500) {
      Serial.println("ACK Timeout");
      return 5;}
    if (i == 4 && ackPacket[3] == 0x00) {
      Serial.println("NAK Received");
      return 1;}}

  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;}
  
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    Serial.println("Success!");
    Serial.print("ACK Received! ");
    //printHex(ackPacket, sizeof(ackPacket));
    return 10;}
  else {
    Serial.print("ACK Checksum Failure: ");
    //printHex(ackPacket, sizeof(ackPacket));
    delay(1000);
    return 1;}
}//end getACK
void readGPS(char* msg, byte size){
  uint msg_index = 0;
  uint item_index = 0;
  uint item_length = 0;
  static char item[20] = {};
  if(*msg == '$' && *(msg+1) == 'G' && *(msg+2) == 'N'){//Looks for start of message with "$GN"
    msg_index += 3;
    if(*(msg+3) == 'R' && *(msg+4) == 'M' && *(msg+5) == 'C'){//Checks message type
      msg_index += 4;//Skips the first comma
      while(msg_index < size){
        if(*(msg+msg_index) == ','){
          switch(item_index){
            case 0: Serial.print("UT:");break;//UNIVERSAL TIME
            case 1: Serial.print("NAV_STAT:");break;//NAVIGATIONAL STATUS
            case 2: Serial.print("LAT:");parseLatLong(item, item_length);break;//LATITUDE
            case 4: Serial.print("LON:");parseLatLong(item, item_length);break;//LONGNITUDE
            case 6: break;//GROUND SPEED
            case 7: break;//COURSE
            case 8: break;//DATE
          }
          for(uint i = 0; i < item_length; i++){
            Serial.print(item[i]);
          }
          Serial.print('-');
          item_index++;
          item_length = 0;
        }else{
          item[item_length] = *(msg+msg_index);
          item_length++;
        }
        msg_index++;
      }
    }
  }
  Serial.println();
}//end readGPS
void parseLatLong(char* value, byte size){//W.I.P.
  static uint hunds, tens, ones;//Converting string to integer
  if(size == 10){//Latitude
    tens = *value - '0'; ones = *(value+1) - '0';//Converting string to integer
    latitude = static_cast<float>(tens*10 + ones);//Converting integer to float
    Serial.println(latitude);
  }
  if(size == 11){//Longitude
    hunds = *value - '0'; tens = *(value+1) - '0'; ones = *(value+2) - '0';//Converting string to integer
    longitude = static_cast<float>((hunds*100) + (tens*10) + ones);//Converting integer to float
    Serial.println(longitude);
  }
}
//============================

//==========RADIO CODE==========Alleon Oxales
char* readyPacket(){//Combines telemetry into bit array then convert to char array
  int bitArray[bitArrayLength] = {};
  static char charArray[charArrayLegnth]  {}; //+1 to include checksum byte, static so that the mem alloc is retained throughout the program
  int bitIndex = 0;
  for(int i = 0; i < 10; i++){
    int dataLength = bitLengthList[i];
    if(i == 1){//Deal with ident
      bitArray[bitIndex] = sender_indent;
      bitIndex++;
    }else if((i == 0) || (i > 1 && i < 8)){//Deal with floats
      float datum = 0;
      switch(i){
        case 0:datum = time_since_launch;break;
        case 2:datum = altitude;break;
        case 3:datum = latitude;break;
        case 4:datum = longitude;break;
        case 5:datum = velocity;break;
        case 6:datum = orientationX;break;
        case 7:datum = orientationY;break;
      }
      int* bitsPtr = dec_to_binary(datum, dataLength);
      for(int x = 0; x < dataLength; x++){
        //Serial.print(*(bitsPtr+dataLength-1-x));
        bitArray[bitIndex] = *(bitsPtr+dataLength-1-x);
        bitIndex++;
      }
    }else if(i == 8){//Deal with error codes
      for(int x = 0; x < dataLength; x++){
        bitArray[bitIndex] = errorCodes[x];
        bitIndex++;
      }
    }else if(i == 9){//Deal with event bits
      for(int x = 0; x < dataLength; x++){
        bitArray[bitIndex] = events[x];
        bitIndex++;
      }
    }
    charArray[charArrayLegnth-3] = radioChecksum(bitArray, bitArrayLength);
    charArray[charArrayLegnth-2] = 'R';
    charArray[charArrayLegnth-1] = 'L';
  }
  int charIndex = 0;
  for(int i = 0; i < bitArrayLength; i+=8){
    char result = 0;
    result |= (bitArray[i] << 7); // Set bit 7
    result |= (bitArray[i+1] << 6); // Set bit 6
    result |= (bitArray[i+2] << 5); // Set bit 5
    result |= (bitArray[i+3] << 4); // Set bit 4
    result |= (bitArray[i+4] << 3); // Set bit 3
    result |= (bitArray[i+5] << 2); // Set bit 2
    result |= (bitArray[i+6] << 1); // Set bit 1
    result |= (bitArray[i+7] << 0); // Set bit 0
    charArray[charIndex] = result;
    charIndex++;
  }
  return charArray;
}//end readyPacket
byte radioChecksum(int *radioMSG, byte msgLength){
  return '_';
}//end radioChecksum
int* dec_to_binary(float my_dec, int my_bit){//Ellie McGshee, Returns elements in reverse order
  int my_dec_int = static_cast<int>(round(my_dec));
  int my_rem;
  static int my_arr[32];
  if (my_bit>32){
    Serial.println("Array requested too large");
    setErr(PRGM_ERR);
  }
  for (int i = my_bit-1; i >= 0; i--){
    int my_exp = pow(2.0, i);
    my_rem = my_dec_int%my_exp;
    if (my_dec_int >= my_exp){
      my_arr[i] = 1;
    }else if (my_dec_int < my_exp){
      my_arr[i] = 0;
    }
    my_dec_int = my_rem;
  }
  return my_arr;
}//end dec_to_binary
//Binary to decimal (Elizabeth McGhee)
float binary_to_dec(int my_bit_size, int* my_arr){
    float my_sum = 0;
    int my_index = 0;
    for (int i = my_bit_size-1; i >= 0; i--){
        my_sum = pow(2, my_index) * *(my_arr+i)+ my_sum;
        my_index++;
    }
    return my_sum;
}//end binary to decimal
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
}//end uint_to_binary
//==============================

//===========EVENT DETECTION CODE==========Elizabeth McGhee
/*bool detect_liftoff(){
  if (altitude > 50.0 and accel.acceleration.y > 2*g){
    return 1;
  }else{
  return 0;}
}
bool detect_burnout(){
  float dummy_variable = 0.3; //We don't know this yet
  float g = 9.81; 
  if (altitude > dummy_variables and accel.acceleration.y < g){
    return 1;
  } else {
    return 0;}
}*/
bool detect_landing(){
  if (altitude < 50.0){
    return 1;
  } else {
  return 0;}  
}//end detect_landing
bool detect_good_shutdown(){//Alleon Oxales
  for(int i = 0; i < 3; i++){
    int value = EEPROM.read(i);
    if(value == 0xff){
      EEPROM.write(i, 0);
    }else{
      return false;
    }
  }
  return true;
}//end detect_good_shutdown
//========================================

//=========SENSOR CODE==========
void TCA9548A(uint8_t bus) {// Select I2C Bus On I2C Multiplexer (if used)
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);            // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}//end TCA9548A
void setupSensors(){//Ryan Santiago
  // 1.) Set accelerometer range:
  /*lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // Other ranges can be set as needed

  // 2.) Set gyroscope range:
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // Other ranges can be set as needed

  // 3.) Set magnetometer range:
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // Other gains can be set as needed*/
}//end setupSensors
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
    return 180;
  }
}
//==============================



void setErr(int errorCode){
  errorCodes[errorCode] = 1;
}//end setErr
void clearErr(int errorCode){
  errorCodes[errorCode] = 0;
}//end clearErr
void commands(int command){
  if(shutdownCheck[2] == 0 && millis() - time_last_command > 1000){
    shutdownCheck[0] = 0;shutdownCheck[1] = 0;shutdownCheck[2] = 0;//Reset shutdown checks after 1s timeout
  }
  switch(command){//Check for commands
    case 0xff:
      for(int i = 0; i < 3; i++){ 
        if(shutdownCheck[i] == 0){
          shutdownCheck[i] = 0xFF;
          break;
        }
      }
      if(shutdownCheck[0] == 0xff && shutdownCheck[1] == 0xff && shutdownCheck[2] == 0xff){//Write shutdown flag to EEPROM
        EEPROM.write(0,0xff);
        EEPROM.write(1,0xff);
        EEPROM.write(2,0xff);
      }
      break;
    case 0x01:
      lowPower = 1;
      break;
    case 0x02:
      lowPower = 2;
      break;
    case 0x03:
      lowDataTransfer = 0;
      break;
    case 0x04:
      lowDataTransfer = 1;
      break;
    case 0x05:
      break;    
  }
  time_last_command = millis();
}
char* checkFile(){//Alleon Oxales
  int fileExists = 1;
  int fileNum = 0;
  static char fileName[8] = "000.txt";
  while(fileExists){
    fileName[0] = '0' + fileNum/100;
    fileName[1] = '0' + (fileNum%100)/10;
    fileName[2] = '0' + fileNum%10;
    File checkfile = SD.open(fileName);
    checkfile.seek(1 * 6);
    String content = checkfile.readStringUntil('\r');
    if(content != ""){
      fileNum++;
    }else{
      fileExists = 0;
    }
  }
  return fileName;
}//end checkFile
