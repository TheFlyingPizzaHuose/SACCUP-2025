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
#include <Adafruit_MPU6050.h> // MPU6050 accelerometer sensor library
#include <Adafruit_Sensor.h> // Adafruit unified sensor library
#include <Adafruit_LSM9DS1.h> // Include LSM9DS1 library
#include <cmath>
#include <iostream>

#define rfSerial Serial1
#define gpsSerial Serial2

// Create the sensor objects
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); //Accelerometer
Adafruit_BMP085 bmp; //Pitot Tube Pressure Sensor

// AV Modes
bool lowPower = 0;
bool lowDataTransfer = 0;
int shutdownCheck[3] = {0,0,0}; //All three elements must be >0 to activate shutdown

int bitLenthList[10] = {12,//Seconds since launch
                        1, //Sender Ident
                        12, //Altitude
                        12, //Latitude
                        12, //Lonitude
                        9, //Velocity
                        8, //OrientationX
                        8, //OrientationY
                        30, //Error Code Array
                        8 //Checksum
                        };//Represents the number of bits for each part of the data transmission

int errorCodes[30] = {}; //Check document for error code list

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

//Sensor measurement time millis WIP
int BMP280_RATE = 7,
    SAMM8Q_RATE = 8,
    MPU6050_RATE = 9,
    BMP180_1_RATE = 10,
    BMP180_2_RATE = 11,
    AS5600_1_RATE = 12,
    AS5600_2_RATE = 13,

//Telemetry variables
float time_since_launch = 0;//seconds
int sender_indent = 0;
float altitude = 0,//meters
      latitude = 0,//meters
      longitude = 0,//meters
      velocity = 0,//meters per second
      orientationX = 0,//degrees
      orientationY = 0;//degrees

void setup() {
  // Start hardware serial communication (for debugging)
  Serial.begin(9600);
  
  rfSerial.begin(57600);//Init RFD UART
  Serial.println("Software serial initialized.");
  
  if (!initSAM_M8Q()) {setErr(SAMM8Q_FAIL);}//Init SAM_M8Q and error if fails
  if (!lsm.begin()) {setErr(MPU6050_FAIL);}//Init MPU6050 and error if fails
  if (!bmp.begin()) {setErr(BMP280_FAIL);}//Init BMP280 and error if fails

  // Setup sensor ranges
  setupSensor();  // Added missing semicolon here
}

void loop() {
  char incomingByte;

  if (rfSerial.available() > 0) {//Ping Data From Ground Station
    incomingByte = rfSerial.read();
    rfSerial.print(incomingByte);
  }

  rfSerial.println(readyPacket());

  // Read sensor data
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
  rfSerial.println();

  delay(50);
}

// Select I2C Bus On I2C Multiplexer (if used)
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);            // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}

void setupSensors(){//Ryan Santiago
  // 1.) Set accelerometer range:
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // Other ranges can be set as needed

  // 2.) Set gyroscope range:
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // Other ranges can be set as needed

  // 3.) Set magnetometer range:
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // Other gains can be set as needed
}

//==========GPS CODE==========//Based on SparkyVT https://github.com/SparkyVT/HPR-Rocket-Flight-Computer/blob/V4_7_0/Main%20Code/UBLOX_GNSS_Config.ino
bool initSAM_M8Q(){
  gpsReady = 1
  gpsSerial.begin(9600);
  //Generate the configuration string for Factory Default Settings
  byte setDefaults[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2F, 0xAE};

  Serial.print("Restoring Factory Defaults... ");
  if(gpsSet(setDefaults) == 3){Serial.println("Restore Defaults Failed!")};

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
  byte set4_1[] = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57};

  //Generate the configuration string for interference resistance settings
  byte setJam[] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45};
  
  //For M8 series gps, just add Galileo, from https://portal.u-blox.com/s/question/0D52p00008HKEEYCA5/ublox-gps-galileo-enabling-for-ubx-m8
  byte setSat[] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x82, 0x56};

  Serial.print("Deactivating NMEA GSA Messages... ");
  if(gpsSet(setGSA) == 3){gpsReady = 0; Serial.println("NMEA GSA Message Deactivation Failed!"):}
  Serial.print("Setting Nav Mode... ");
  if(gpsSet(setNav) == 3){gpsReady = 0; Serial.println("Nav mode Failed!"):}
  Serial.print("Deactivating NMEA GSV Messages... ");
  if(gpsSet(setGSV) == 3){gpsReady = 0; Serial.println("NMEA GSV Message Deactivation Failed!");}
  Serial.print("Setting 10Hz data rate... ");
  if(gpsSet(setDataRate) == 3){gpsReady = 0; Serial.println("Set 10Hz data rate Failed!");}
  Serial.print("Activating NMEA 4.1 Messages... ");
  if(gpsSet(set4_1) == 3){gpsReady = 0; Serial.println("NMEA 4,1 Activation Failed!");}
  Serial.print("Set Satellite Constelations... ");
  if(gpsSet(setSat) == 3){gpsReady = 0; Serial.println("Satellite Setting Failed!");}
  Serial.print("Deactivating NMEA VTG Messages... ");
  if(gpsSet(setVTG) == 3){gpsReady = 0; Serial.println("NMEA VTG Message Deactivation Failed!");}
  Serial.print("Setting Interference Threshols... ");
  if(gpsSet(setJam) == 3){gpsReady = 0; Serial.println("Interference Settings Failed!");}
  Serial.print("Deactivating NMEA GLL Messages... ");
  if(gpsSet(setGLL) == 3){gpsReady = 0; Serial.println("NMEA GLL Message Deactivation Failed!");}
  
  //Increase Baud-Rate on M8Q for faster GPS updates
  setSucess = 0
  while(gpsVersion == 2 && setSucess < 3) {
    Serial.print("Setting Ublox Baud Rate 38400... ");
    sendUBX(setBaudRate, sizeof(setBaudRate));
    setSucess += getUBX_ACK(&setBaudRate[2]);}
  if (setSucess == 3 ){gpsReady = 0; Serial.println("Ublox Baud Rate 38400 Failed!");}
  else if(gpsVersion == 2){
    HWSERIAL->end();
    HWSERIAL->flush();
    HWSERIAL->begin(38400);}
  return gpsReady;
}//end ConfigGPS
int gpsSet(byte msg, char){
  int gpsSetSuccess = 0;
  while(gpsSetSuccess < 3) {
      sendUBX(msg, sizeof(msg));
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
    gpsSerial.flush();}
    
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
    printHex(ackPacket, sizeof(ackPacket));
    return 10;}
  else {
    Serial.print("ACK Checksum Failure: ");
    printHex(ackPacket, sizeof(ackPacket));
    delay(1000);
    return 1;}
}//end getACK
void readGPS(){//W.I.P.
  
}//end readGPS
//============================

//==========RADIO CODE==========Alleon Oxales
char* readyPacket(){//Combines telemetry into bit array then convert to char array
  const int bitArrayLength = 104;
  const int charArrayLegnth = bitArrayLength/8;
  int bitArray[bitArrayLength] = {};
  char charArray[]  {};
  int bitIndex = 0;
  if(lowDataTransfer){
    for(int i = 0; i < 10; i++){
      int dataLength = bitLenthList[i];
      if(i == 1){//Deal with ident
        bitArray[bitIndex] = 0;//0 for avionics
      }else if((i == 0) || (i > 1 && i < 8)){//Deal with floats
        float datum = 0;
        switch(i){
          case 0;datum = time_since_launch;break;
          case 2:datum = altitude;break;
          case 3:datum = latitude;break;
          case 4:datum = longitude;break;
          case 5:datum = velocity;break;
          case 6:datum = orientationX;break;
          case 7:datum = orientationY;break;
        }
        int* bitsPtr = dev_to_binary(datum, dataLength);
        for(int x = 0; x < dataLength; x++){
          bitArray[bitIndex] = *(bitsPtr+dataLength-1-x);
          bitIndex++;
        }
      }else if(i == 8){//Deal with error codes and checksum
        for(int x = 0, x < dataLength; x++){
          bitArray[bitIndex] = errorCodes[x];
        }
      }else if(i == 9){
        byte checksum = radioChecksum(bitArray, bitArrayLength);
      }
    }
    charIndex = 0;
    for(int i = 0; i < bitArrayLength; i+=8){
      char result = 0;
      result |= (bitArray[i] << 7); // Set bit 7
      result |= (bitArray[i+1] << 6); // Set bit 6
      result |= (bitArray[i+2] << 5); // Set bit 5
      result |= (bitArray[i+3] << 4); // Set bit 4
      result |= (bitArray[i+4] << 3); // Set bit 3
      result |= (bitArray[i+5] << 2); // Set bit 2
      result |= (bitArray[i+6]6 << 1); // Set bit 1
      result |= (bitArray[i+7]7 << 0); // Set bit 0
      charArray[charIndex] = result;
      charIndex++;
    }
    return charArray;
  }
}
byte radioChecksum(int *radioMSG, byte msgLength){
  
}
//==============================

void setErr(int errorCode){
  errorCodes[errorCode] = 1;}

void clearErr(int errorCode){
  errorCodes[errorCode] = 0;}

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
}//dec_to_binary