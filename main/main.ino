/*
 * Portable Solar Tracking System - Main
 * 
 * This program will contain the main control for the two-axis Photovoltaic (PV)
 * tracking system. The Arduino will communicate with the stepper motor drives,
 * a 16-bit ADC, hall effect sensors, and any other peripheral devices deemed
 * necessary. 
 */

 #include <DRV8834.h>
 #include <Adafruit_ADS1115.h> //Custom ADC library for ADS1115 provided by Adafruit

const int STEPS_PER_REV = 200; //Both motors have 200 steps per revolution
//Degrees per step: 360/200 = 1.8

/*
 * Analog to Digital converter module uses I2C interface.
 * Pin Configuration: 
 * SDA pin -> A4/18/SDA (on Arduino Nano)
 * SCL pin -> A5/19/SCL (on Arduino Nano)
 * ADDR pin -> GND (on ADS1115). Set default I2C address to 1001 000
 * Read analog channels between 0-3 with readADC_SingleEnded(#)
 */
Adafruit_ADS1115 adc;

/*
 * Analog channels for solar sensors light intensity
 * Top - 0, Right - 1, Bottom - 2, Left - 3
 */
int top=0;
int right=1;
int bottom=2;
int left=3;

//Digital output values from ADS module
int16_t topSensor, rightSensor, bottomSensor, leftSensor; 

int dirPin1 = 2; //Arduino pin, direction for first driver
int stepPin1 = 3; //Arduino pin, step pulse output to first driver
int dirPin2 = 4; //Arduino pin, direction for second driver
int stepPin2 = 5; //Arduino pin, step pulse output to second driver

//Instantiate both stepper motor driver objects
DRV8834 stepperTheta(STEPS_PER_REV, dirPin1, stepPin1); //Driver that controls rotation motor
DRV8834 stepperPhi(STEPS_PER_REV, dirPin2, stepPin2); //Driver that controls tilt motor

void setup() {
  //Set stepper motor speeds at 1 rpm
  stepperTheta.setRPM(1);
  stepperPhi.setRPM(1);

  //Intialize ADS1115
  adc.begin();
  //Intialize serial port for debugging purposes
  Serial.begin(9600);
}

void loop() {
  //Read sensor values
  topSensor = adc.readADC_SingleEnded(top);
  bottomSensor = adc.readADC_SingleEnded(bottom);
  leftSensor = adc.readADC_SingleEnded(left);
  rightSensor = adc.readADC_SingleEnded(right);

  Serial.print("Top: "); Serial.println(topSensor);
  Serial.print("Bottom: "); Serial.println(bottomSensor);
  Serial.print("Left: "); Serial.println(leftSensor);
  Serial.print("Right: "); Serial.println(rightSensor);
  delay(1000); //delay one second
}
