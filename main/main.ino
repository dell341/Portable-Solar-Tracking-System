/*
 * Portable Solar Tracking System - Main
 * 
 * This program will contain the main control for the two-axis Photovoltaic (PV)
 * tracking system. The Arduino will communicate with the stepper motor drives,
 * a 16-bit ADC, hall effect sensors, and any other peripheral devices deemed
 * necessary. 
 */

#include <DRV8834.h>
#include <Adafruit_ADS1015.h> //Custom ADC library for ADS1115 provided by Adafruit

/*
 * FOR DEBUGGING PURPOSES ONLY
 */
//Declare characters that will be used to parse instructions
const bool DEBUG = false;
const char endStatement = ';';
const char delimeter = '.';

String instruction; //Complete instruction separated by semicolons
char command; //Command portion of instruction
String params; //Parameters portion of instruction
/*
 * DEBUGGING END
 */

const int STEPS_PER_REV = 200; //Both motors have 200 steps per revolution. Degrees per step = 1.8
//Voltage threshold determines whether the summation of all sensors is enough to start tracking. Basically Day/Night sensor
/*
 * TODO: Find real value for voltage threshold
 * TODO: Find real value for max phi. Determine if it will be in steps or degrees.
 * TODO: Find real value for voltage difference
 */
const float VOLTAGE_THRESHOLD = 0.5; 
const float VOLTAGE_DIFF = 0.2; //Deadband for differences in voltage levels between sensors. Margin where nothing is changed
const int MAX_PHI = 100; //Maximum position for phi on both sides of the solar panel. +max_phi(degrees) and -max_phi(degrees)
const int STEP_PHI = 25; //Amount of steps for each phi movement
const int STEP_THETA = 50; //Amount of steps for each theta movement
const unsigned long THIRTY_MINUTES_MILLIS = 1800000; //30 minutes represented in milliseconds. Used for sleep mode operations

bool isThetaSleeping; //Returns whether or not motor drivers are currently in sleep mode
bool isPhiSleeping; 
bool initiatedSleepMode = false; //Will be used for extending sleeping periods, 30 minutes
unsigned long lastMeasuredTime;

int dirPin1 = 2; //Arduino pin, direction for first driver
int stepPin1 = 3; //Arduino pin, step pulse output to first driver
int sleepPin1 = 4; //Arduino pin, sleep/enable pin for the first driver
int dirPin2 = 5; //Arduino pin, direction for second driver
int stepPin2 = 6; //Arduino pin, step pulse output to second driver
int sleepPin2 = 7; //Arduino pin, sleep/enable pin for the second driver

int upperLimitPin = 10; //Arduino pin, hall sensor - solar panel phi position upper limit
int homePin = 11; //Arduino pin, hall sensor - solar panel home position 
int lowerLimitPin = 12; //Arduino pin, hall sensor - solar panel phi position lower limit

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
 * Top - 0, Bottom - 1, Left - 2, Right - 3
 */
int top=0;
int bottom=1;
int left=2;
int right=3;
int count = 0; //Counts the amount times solar sensors are below threshold

//Digital output values from ADS module
float topSensor, rightSensor, bottomSensor, leftSensor; 

//Instantiate both stepper motor driver objects
DRV8834 stepperTheta(STEPS_PER_REV, dirPin1, stepPin1, sleepPin1); //Driver that controls rotation motor
DRV8834 stepperPhi(STEPS_PER_REV, dirPin2, stepPin2, sleepPin2); //Driver that controls tilt motor

//Keeps track of motor position based on steps taken
int phiPosition = 0;
int thetaPosition = 0;

void setup() {
  //Set stepper motor speeds
  stepperTheta.setRPM(25);
  stepperPhi.setRPM(25);

  //Intially put both motor drivers in sleep mode
  sleepMode();

  //Set intial read value of sensors to 0
  topSensor = 0;
  rightSensor = 0;
  bottomSensor = 0;
  leftSensor = 0;

  //Set pin modes
  pinMode(upperLimitPin,INPUT);
  pinMode(homePin,INPUT);
  pinMode(lowerLimitPin,INPUT);

  //Intialize ADS1115
  adc.setGain(GAIN_ONE); //Sets the voltage output range to +/- 4.096V. 1 bit = 0.125mV
  adc.begin();
  //Intialize serial port for debugging purposes
  Serial.begin(9600);

  rightSensor = 2.0;
  leftSensor = 0.15;
  wakeMode();
}

void loop() {
  if(DEBUG) {
    if(Serial.available()) {
      //Separates instructions by end-statement characters
      instruction = Serial.readStringUntil(endStatement); 
  
      int i1 = instruction.indexOf(delimeter);//Grabs position of first delimeter
      command = instruction.charAt(0); //Command should be first index
      params = instruction.substring(i1+1); //Parameters should be after first delimeter
  
      switch(command) {
        case 'm': //Moves motor up or down
          commandMove(params);
        break;
        case 's': //Steps motor up or down
          commandStep(params);
        break;
        case 'w': //Wakes motor drivers
          wakeMode();
        break;
        case 'z': //Puts motor drivers to sleep
          sleepMode();
        break;
        default:
          Serial.println("Invalid instruction received");
        break;
      }
    }
  }
  else {
    //readSensors();
    
    Serial.print("Top: "); Serial.println(topSensor);
    Serial.print("Bottom: "); Serial.println(bottomSensor);
    Serial.print("Left: "); Serial.println(leftSensor);
    Serial.print("Right: "); Serial.println(rightSensor);
    Serial.println();
    
   if(rightSensor-leftSensor > VOLTAGE_DIFF) {
    Serial.println("Would send command to step theta down");
    //stepThetaUp();
    rightSensor = rightSensor - 0.2;
   }
   else if(leftSensor-rightSensor > VOLTAGE_DIFF) {
    Serial.println("Would send command to step theta up");
    
   }
   else {
      if(!isPhiSleeping) {
        sleepMode();
      }
   }

   delay(1500);

  }
}

/*
 * Steps specified motor up or down
 * instruction: s.[motor].[direction]
 * motor is either t (theta) or p (phi)
 * direction is either u (up) or d (down)
 */
void commandStep(String params) {
  int i1 = params.indexOf(delimeter); //Find index of first delimeter

  char m = params.charAt(0); //selected motor
  char d = params.charAt(i1+1); //selected direction

  Serial.print("Step motor by ");
  Serial.print(STEP_PHI);
  Serial.println();
  switch(m) {
    case 't':
      if(d == 'u')
        stepThetaUp();
      else if(d == 'd')
        stepThetaDown();
      break;
     case 'p':
      if(d == 'u')
        stepPhiUp();
      else if(d == 'd')
        stepPhiDown();
      break; 
     default:
      Serial.println("Invalid instruction received");
      break;
  }
}

/*
 * Move specified motor up or down
 * instruction: m.[motor].[amount]
 * motor is either t (theta) or p (phi)
 * amount is amount of steps
 */
void commandMove(String params) {
  int i1 = params.indexOf(delimeter); //Find index of first delimeter

  char m = params.charAt(0); //selected motor
  int amount = params.substring(i1+1).toInt(); //selected direction

  Serial.print("Move motor by ");
  Serial.print(amount);
  Serial.println();
  switch(m) {
    case 't':
        moveThetaBy(amount);
      break;
    case 'p':
        movePhiBy(amount);
      break;
    default:
     Serial.println("Invalid instruction received");
     break;
  }
}

/*
 * Update values of all solar sensors
 * returns: void (nothing)
 */
 void readSensors() {

  int16_t maxValue = 2^15; //15 bit- resolution
  int maxVoltage = 4.096; //Gain set from ADC
  
  //Read single value of sensors from each ADC analog channel
  topSensor = adc.readADC_SingleEnded(top)*(maxVoltage/maxValue);
  bottomSensor = adc.readADC_SingleEnded(bottom)*(maxVoltage/maxValue);
  leftSensor = adc.readADC_SingleEnded(left)*(maxVoltage/maxValue);
  rightSensor = adc.readADC_SingleEnded(right)*(maxVoltage/maxValue);
 }

 /*
  * Verifies whether summation of sensor values are above defined voltage threshold
  * returns: true or false
  */
boolean isAboveThreshold() {
  int summation = topSensor + bottomSensor + leftSensor + rightSensor;

  if(summation < VOLTAGE_THRESHOLD)
         count++;
     else
        count=0;
      
  return summation >= VOLTAGE_THRESHOLD;
}
   
/*
*Function that counts isAboveThreshold three consecutive times
* Returns true if solar sensors have been below the threshold for the past three consecutive reads
* returns: true or false
*/
boolean isBelowThreeTimes(){
 if(count>=3){
   return true;
 }else
   return false;
}

/*
 * Measures the voltage values between both the current sensor and the target sensor.
 * Returns true if the difference between the sensors is greater than the deadband
 * returns: true or false
 */
boolean isAboveDiff(int currentSensor, int targetSensor) {
  int difference = currentSensor - targetSensor;
  return difference > VOLTAGE_DIFF;
}

/*
 * Determines whether solar panel is at the physical low limit
 * Returns true if hall sensor reads low, false if hall sensor reads high
 * returns: true or false
 */
 boolean isAtMin() {
  return !digitalRead(lowerLimitPin);
 }

/*
 * Determines whether solar panel is at home/resting position. 
 * Home position is when the panel is completely horizontally flat. Phi = 0.
 * Returns true if hall sensor reads low, false if hall sensor reads high
 * returns: true or false
 */
boolean isAtHome() {
  return !digitalRead(homePin);
}

/*
 * Determines whether solar panel is at the physical upper limit
 * Returns true if hall sensor reads low, false if hall sensor reads high
 * returns: true or false
 */
 boolean isAtMax() {
  return !digitalRead(upperLimitPin);
 }

/*
 * Solar panel goes to home position (horizontally flat).
 * After home position is reached, current positon is set to zero
 * returns: void
 */
void goToHome() {
  if(isAtHome()) {//If solar panel is already at home position, leave function
    phiPosition = 0;
    return;
  }

    //First increase phi to find home, stop if upper limit sensor is reached
    while(!isAtHome() || !isAtMax())
      stepPhiUp();

  //If home position was reached leave function, else move the other direction
    if(isAtHome()) {
      phiPosition = 0;
      return;
    }

  //Decrement phi to find home, stop if lower limit sensor is reached
    while(!isAtHome() || !isAtMin())
      stepPhiDown();
}

/*
 * Command the phi driver to move the motor by the specified amount
 * returns: void
 */
void movePhiBy(int amountToMove) {
  //For now let's set limits to 100 steps, until stepping mode can be determined
  //if(abs(amountToMove) > 100)
    //return;

  Serial.println("Moving phi");
  stepperPhi.move(amountToMove);
  phiPosition = phiPosition + amountToMove; //Update phi position
}

/*
 * Command the phi driver to step up by predefined amount
 * returns: void
 */
 void stepPhiUp() {
  Serial.println("Stepping phi up");
    stepperPhi.move(STEP_PHI);
 }

 /*
  * Comand the phi driver to step down by predefined amount
  */
  void stepPhiDown() {
    Serial.println("Stepping phi down");
      stepperPhi.move(-STEP_PHI);
  }

/*
 * Determines whether or not phi motor can be moved by step amount
 * Returns false if difference between max phi position and current phi position is less than step_phi
 * returns: true or false
 */
 boolean canMovePhi() {
  return (MAX_PHI - abs(phiPosition)) > STEP_PHI;
 }
 
/*
 * Command the theta driver to move the motor by the specified amount
 * returns: void
 */
void moveThetaBy(int amountToMove) {
  //For now let's set limits to 200 steps, until stepping mode can be determined
  if(abs(amountToMove) > 200)
    return;

  Serial.println("Moving theta");
  stepperTheta.move(amountToMove);
  thetaPosition = thetaPosition + amountToMove; //Update theta position
}

/*
 * Command the theta driver to step up by predefined amount
 * returns: void
 */
 void stepThetaUp() {
    Serial.println("Stepping theta up");
    stepperTheta.move(STEP_THETA);
 }

 /*
  * Comand the Theta driver to step down by predefined amount
  */
  void stepThetaDown() {
    Serial.println("Stepping theta down");
      stepperTheta.move(-STEP_THETA);
  }

  /*
   * Power down the motor drivers to limit power consumption
   * Disables control to both motor drivers
   * selectedDriver : t (for theta driver), p (for phi driver)
   * If no input is given, both motors are put to sleep
   */
  void sleepMode() {
    sleepMode('n');
  }
  
  void sleepMode(char selectedDriver) {
    if(selectedDriver == 't') {
      stepperTheta.sleep();
      isThetaSleeping = true;
      
      Serial.println("Theta motor driver in sleep mode");
    }
    else if(selectedDriver == 'p') {
      stepperPhi.sleep();
      isPhiSleeping = true;
      
      Serial.println("Phi motor driver in sleep mode");
    }
    else {
      stepperTheta.sleep();
      stepperPhi.sleep();
      isThetaSleeping = true;
      isPhiSleeping = true; 
      
      Serial.println("Both motor drivers in sleep mode");
    }
  }

  /*
   * Power up the motor drivers, back to ready state
   * Re-enables control back to both motors
   * selectedDriver : t (for theta driver), p (for phi driver)
   * If no input is given, both motors are awaken
   */
  void wakeMode() {
    wakeMode('n');
  }
  
  void wakeMode(char selectedDriver) {
    if(selectedDriver == 't') {
      stepperTheta.wake();
      isThetaSleeping = false;
      
      Serial.println("Theta motor driver in wake mode");
    }
    else if(selectedDriver == 'p') {
      stepperPhi.wake();
      isPhiSleeping = false;
      
      Serial.println("Phi motor driver in wake mode");
    }
    else {
      stepperTheta.wake();
      stepperPhi.wake();
      isThetaSleeping = false;
      isPhiSleeping = false; 
      
      Serial.println("Both motor drivers in wake mode");
    }
  }

/*
 * After solar panel has reached its desired position,
 * it will be 30 minutes
 */
  void sleepFor30() {
    initiatedSleepMode = true;
    sleepMode();
    lastMeasuredTime = millis(); //Gets time (in milliseconds) that sleep mode is initated
  }

  /*
   * Returns true if 30 minutes has passed since sleep mode has been initiated
   */
   bool hasSleptFor30() {
    //If sleep mode was never initated or one of the motors is awake, 
    if(!initiatedSleepMode || !isThetaSleeping || !isPhiSleeping)
      return false;

      long currentTime = millis();
      long elapsedTime = lastMeasuredTime - currentTime;

      lastMeasuredTime = currentTime;

      if(elapsedTime >= THIRTY_MINUTES_MILLIS)
        return true;
      else
        return false;
   }

