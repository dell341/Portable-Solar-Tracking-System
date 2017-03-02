/*
 * Portable Solar Tracking System - Main
 * 
 * This program will contain the main control for the Photovoltaic (PV)
 * tracking system. The Arduino will communicate with the stepper motor drives,
 * a 16-bit ADC, hall effect sensors, and any other peripheral devices deemed
 * necessary. 
 */

 #include <Stepper.h> //Stepper motor driver library provided by Arduino
 #include <Adafruit_ADS1115.h> //Custom ADC library for ADS1115 provided by Adafruit

const int stepsPerRev = 200; //Both motors have 200 steps per revolution
//Degrees per step: 360/200 = 1.8

//Instantiate both stepper motor objects
Stepper stepperTheta(stepsPerRev, 2, 3, 4, 5); //Connect theta stepper to digital pins 2,3,4 and 5
Stepper stepperPhi(stepsPerRev, 6, 7, 8, 9); //Connect phi stepper to digital pins 6,7,8 and 9

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

void setup() {
  //Set stepper motor speeds at 60 rpm
  stepperTheta.setSpeed(60);
  stepperPhi.setSpeed(60);

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
