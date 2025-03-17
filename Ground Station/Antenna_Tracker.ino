#include <AccelStepper.h>
#include <Adafruit_LSM9DS1.h>
#include <math.h>
#include <Wire.h>

#define LSM9DS1_ADDRESS 0x6B  // I2C address of LSM9DS1 (could be 0x6A or 0x6B)
AccelStepper stepper1(1, 2, 4); // (Type of driver: with 2 pins, STEP, DIR)

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // i2c sensor

long max_step_speed = 500000;
int position = 0, max_pos = -530;
int button1State = 0, button2State = 0;
int button1 = 4, button2 = 3;

int pos_change_del = 100;
int last_pos_change = 0;
const float up[3] = {0,0,1};

void setup() {
  Serial.begin(9600);
  if(lsm.begin())
  {
    Serial.print("LSM OK.");
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_952HZ);
  //pinMode(button1, INPUT_PULLUP);
  //pinMode(button2, INPUT_PULLUP);

  // Set maximum speed value for the stepper   
  stepper1.setMaxSpeed(10000000);
  stepper1.setAcceleration(100);
  stepper1.setCurrentPosition(0);
}
void loop() {
  //Serial.println(position);
  button1State = digitalRead(button1);
  button2State = digitalRead(button2);
  if(last_pos_change - millis() >= pos_change_del){
    sensors_event_t a, gyro, mag, temp;
    lsm.getEvent(&a, &gyro, &mag, &temp);
    float accel[3] = {a.acceleration.x,a.acceleration.y,a.acceleration.z};
    float theta = floor(angle(accel, up));
    position = (int)round(max_pos*theta/80);
    if(position < max_pos){
      position = max_pos;
    }
    last_pos_change = millis();
    stepper1.moveTo(position);
  }
  stepper1.run();
}

float dot2(float a[3], float b[3]){return a[1]*b[1] + a[2]*b[2];}

float mag2(float a[3]){return sqrt(a[1]*a[1] + a[2]*a[2]);}

float angle(float a[3], float b[3]){
  if(a[1] < 0){
    return acos(dot2(a,b) / (mag2(a) * mag2(b))) * 180/3.141592;
  }else{
    return -acos(dot2(a,b) / (mag2(a) * mag2(b))) * 180/3.141592;
  }
}
