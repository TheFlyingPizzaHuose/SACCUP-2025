/*   
 *   Basic example code for controlling a stepper with the AccelStepper library
 *      
 *   by Dejan, https://howtomechatronics.com
 */

#include <AccelStepper.h>
#include <SoftwareSerial.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 3); // (Type of driver: with 2 pins, STEP, DIR)
const int max = 2500;
int lastTime;

void setup() {
  // Set maximum speed value for the stepper
  stepper1.setMaxSpeed(max);
  Serial.begin(9600);
  lastTime = millis();
}

int i = 1;
int accel = 1;
void loop() {
  stepper1.setSpeed(i);
  stepper1.runSpeed();
  if(millis()-lastTime > 1){
    i+=accel;
    lastTime = millis();
  //Serial.println(String(i));
  }
  if(i == max){
    accel = 0;
  }
  // Step the motor with a constant speed previously set by setSpeed();
}