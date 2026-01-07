/*
  Braitenberg-Lab01.ino
  Ethan Harden
  Alex Stedman
  Alex Yim
  12/12/25

  This program will introduce using the stepper motor library to create motion algorithms for the robot, for lab 1.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and basic movements (stop, forward, spin, reverse, turn)
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  Arduino pin mappings: https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet#pins
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182 

  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor
  digital pin 4 - blue LED in series with 220 ohm resistor

  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin

  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include "RPC.h" //for other core


//imu
#include <MPU6050.h> 
#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define bluLED 4            //blue LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[4] = {redLED,grnLED,ylwLED,bluLED};      //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define max_speed 1500 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration

int pauseTime = 2500;   //time before robot moves in ms
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 3000;   //delay for printing data

#define TRACKWIDTH 216   //distance between the wheels in mm
#define MOVE_VEL 100     //velocity of movement, mm/s
#define ROT_VEL 1     //velocity of movement, rad/s


#define RAND_FLOAT_STEP_WIDTH 100     //makes it so you get random to 0.01 place

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset



#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 11

// old sonars
#define leftSnr 2
#define rightSnr 3

// new sonars
#define purpleSnrTrig 31 //digital
#define purpleSnrEcho 33 //digital
#define orangeSnrTrig 30 //digital
#define orangeSnrEcho 32 //digital

#define numLidars 4
int lidars[numLidars] = {frontLdr, backLdr, leftLdr, rightLdr};
unsigned long lidarRisingTimes[numLidars] = {0, 0, 0, 0};
#define lidarDisconnectTimeout 40000
uint8_t lidarStates[numLidars] = {0, 0, 0, 0}; //either 0 (low) or 1 (high)
#define LIDAR_FAR_THRESH 60

#define numSonars 2
int sonars[numSonars] = {leftSnr, rightSnr};
unsigned long sonarTimes[numSonars] = {0, 0};
#define sonarTriggerDelay 100 // example code had 10: for how long the trigger (putting a bit of sound in the room to measure with) is
#define sonarAfterReadDelay 40000 //higher number means less interference between the two sonars (and between two reads from the same sonar), lower number means more frequent sensor reads. Big enough number (400000) means the alternating sensor lights is visible.
#define sonarStartTimeout 40000
int sonarStates[numSonars] = {0, 0};//0: do trigger start, 1: waiting sonarTriggerDelay micros, 2: reading, 3: waiting sonarAfterReadDelay micros

#define numNewSonars 2
int newSonarTriggers[numNewSonars] = {purpleSnrTrig, orangeSnrTrig};//digital
int newSonarEchos[numNewSonars] = {purpleSnrEcho, orangeSnrEcho};//digital
unsigned long newSonarTimes[numNewSonars] = {0, 0};
#define newSonarTriggerDelay 10 // data sheet says 10: for how long the trigger (putting a bit of sound in the room to measure with) is
#define newSonarAfterReadDelay sonarAfterReadDelay //reusing the same constant
#define newSonarStartTimeout sonarStartTimeout
int newSonarStates[numNewSonars] = {0, 0};//0: do trigger start, 1: waiting sonarTriggerDelay micros, 2: reading, 3: waiting sonarAfterReadDelay micros


// imu
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Helper Functions



struct sensors {
  float lidars[numLidars];
  float sonars[numSonars];
  float newSonars[numNewSonars];
  // this defines some helper functions that allow RPC to send our struct (I [Berry] found this on a random forum)
  MSGPACK_DEFINE_ARRAY(lidars, sonars, newSonars);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} sense;



// how many times a sensor needs to not read in order to set the distance to far away
#define TIMES_NO_READ_THRES 10

// how far is far away when a sensor doesn't read
#define NO_READ_DIST 1000

// when a sensor sucessfully reads, the corresponding sensor gets set to 0, each time it doesn't read it increments by one
struct timesNoReadStruct {
  unsigned int lidars[numLidars];
  unsigned int sonars[numSonars];
  unsigned int newSonars[numNewSonars];

  // doesn't need to be sent across processors
  // MSGPACK_DEFINE_ARRAY(lidar_front, lidar_back, lidar_left, lidar_right, sonar_left, sonar_right);
} timesNoRead;


// read_lidars is the function used to get lidar data to the M7
struct sensors read_sensors() {
  return sense;
}

// micros to some unit (maybe cm)
float lidarTimeToDist(float t) {
  return (t - 1000.0f) * 3.0f / 40.0f;
}

// micros to some unit (maybe cm)
float sonarTimeToDist(float t){
  return t * ((331.5f + 0.6f * 20) * 100 / 1000000.0) / 2;
}

// micros to cm, from datasheet https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
float newSonarTimeToDist(float t){
  return t/58;
}


void initLidar(int lidarIndex) {
  timesNoRead.lidars[lidarIndex] = TIMES_NO_READ_THRES;
}

void initSonar(int sonarIndex) {
  timesNoRead.sonars[sonarIndex] = TIMES_NO_READ_THRES;
}

void initNewSonar(int sonarIndex) {
  timesNoRead.newSonars[sonarIndex] = TIMES_NO_READ_THRES;

  pinMode(newSonarTriggers[sonarIndex], OUTPUT);
  digitalWrite(newSonarTriggers[sonarIndex], LOW);  

  pinMode(newSonarEchos[sonarIndex], INPUT);
}

//set up the M4 (coprocessor) to be sensor server
void setupM4() {
  RPC.bind("read_sensors", read_sensors);  // bind a method to return the sensor data all at once

  // imu
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  accelgyro.initialize();

  for(int i=0; i<numLidars; i++)
    initLidar(i);
  for(int i=0; i<numSonars; i++)
    initSonar(i);
  for(int i=0; i<numNewSonars; i++)
    initNewSonar(i);
}


// lidar is easier
void readLidar(int lidarIndex){
  int state = digitalRead(lidars[lidarIndex]);
  bool noRead = false;
  if(lidarStates[lidarIndex] ^ state){ //xor: not the same
    if(state) {
      //now high: rising edge
      lidarRisingTimes[lidarIndex] = micros();
    } else {
      // falling edge
      float dist = lidarTimeToDist(micros()-lidarRisingTimes[lidarIndex]);
      if(dist<LIDAR_FAR_THRESH) {
        sense.lidars[lidarIndex] = dist;
        timesNoRead.lidars[lidarIndex] = 0;
      } else {
        noRead = true;
      }
    }
  } else if(lidarRisingTimes[lidarIndex]+lidarDisconnectTimeout<=micros()){ //no change for too long
    // get a strike, but don't change any pins to give up read, also not resetting the timer so this gets all strikes pretty quickly
    noRead = true;
  }

  if(noRead) {
    if(timesNoRead.lidars[lidarIndex] == TIMES_NO_READ_THRES) {
      sense.lidars[lidarIndex] = NO_READ_DIST;
    } else {
      timesNoRead.lidars[lidarIndex]++;
    }
  }

  lidarStates[lidarIndex] = state;
}

// sonar is harder because the pin is used for triggering and reading
// sonarStates:  0: do trigger start, 1: waiting sonarTriggerDelay micros, 2: waiting for read to start, 3: reading, 4: waiting sonarAfterReadDelay micros
// returns true if the next (the other because there is only 2) sonar should be read
bool readSonar(int sonarIndex){
  int read = digitalRead(sonars[sonarIndex]);

  if(sonarStates[sonarIndex]==4 && (sonarTimes[sonarIndex]+sonarAfterReadDelay<=micros())){
    // done waiting after read, able to start the trigger
    sonarStates[sonarIndex] = 0;

    // stop putting sound in the room (not entirely sure this is doing anything)
    pinMode(sonars[sonarIndex], OUTPUT);
    digitalWrite(sonars[sonarIndex], LOW);    

    // actually alternate
    return true;//stays 0 until next time
  }

  if(sonarStates[sonarIndex]==3 && !read){
    // end of read
    sense.sonars[sonarIndex] = sonarTimeToDist(micros()-sonarTimes[sonarIndex]);
    timesNoRead.sonars[sonarIndex] = 0;
    sonarStates[sonarIndex] = 4;
    sonarTimes[sonarIndex] = micros();
  }

  if(sonarStates[sonarIndex]==2){
    if(read){
      // start of read
      sonarStates[sonarIndex] = 3;
      sonarTimes[sonarIndex] = micros();
    } else if(sonarTimes[sonarIndex]+sonarStartTimeout<=micros()) {
      // give up read
      sonarStates[sonarIndex] = 4;
      sonarTimes[sonarIndex] = micros();
      if(timesNoRead.sonars[sonarIndex] == TIMES_NO_READ_THRES) {
        sense.sonars[sonarIndex] = NO_READ_DIST;
      } else {
        timesNoRead.sonars[sonarIndex]++;
      }
    }
  }

  if(sonarStates[sonarIndex]==1 && (sonarTimes[sonarIndex]+sonarTriggerDelay<=micros())){
    // finish trigger, start read
    digitalWrite(sonars[sonarIndex], LOW);
    pinMode(sonars[sonarIndex], INPUT);
    sonarStates[sonarIndex] = 2;
    sonarTimes[sonarIndex] = micros();
  }

  if(sonarStates[sonarIndex]==0){
    // start trigger
    pinMode(sonars[sonarIndex], OUTPUT);
    digitalWrite(sonars[sonarIndex], LOW);
    digitalWrite(sonars[sonarIndex], HIGH);
    sonarStates[sonarIndex] = 1;
    sonarTimes[sonarIndex] = micros();
  }

  return false;
}


bool readNewSonar(int sonarIndex){
  int read = digitalRead(newSonarEchos[sonarIndex]);

  if(newSonarStates[sonarIndex]==4 && (newSonarTimes[sonarIndex]+newSonarAfterReadDelay<=micros())){
    // done waiting after read, able to start the trigger
    newSonarStates[sonarIndex] = 0;

    // actually alternate
    return true;//stays in state 0 until next time
  }

  if(newSonarStates[sonarIndex]==3 && !read){
    // end of read
    sense.newSonars[sonarIndex] = newSonarTimeToDist(micros()-newSonarTimes[sonarIndex]);
    timesNoRead.newSonars[sonarIndex] = 0;
    newSonarStates[sonarIndex] = 4;
    newSonarTimes[sonarIndex] = micros();
  }

  if(newSonarStates[sonarIndex]==2){
    if(read){
      // start of read
      newSonarStates[sonarIndex] = 3;
      newSonarTimes[sonarIndex] = micros();
    } else if(newSonarTimes[sonarIndex]+newSonarStartTimeout<=micros()) {
      // give up read
      newSonarStates[sonarIndex] = 4;
      newSonarTimes[sonarIndex] = micros();
      if(timesNoRead.newSonars[sonarIndex] == TIMES_NO_READ_THRES) {
        sense.newSonars[sonarIndex] = NO_READ_DIST;
      } else {
        timesNoRead.newSonars[sonarIndex]++;
      }
    }
  }

  if(newSonarStates[sonarIndex]==1 && (newSonarTimes[sonarIndex]+newSonarTriggerDelay<=micros())){
    // finish trigger, start read
    digitalWrite(newSonarTriggers[sonarIndex], LOW);
    newSonarStates[sonarIndex] = 2;
    newSonarTimes[sonarIndex] = micros();
  }

  if(newSonarStates[sonarIndex]==0){
    // start trigger
    pinMode(newSonarTriggers[sonarIndex], OUTPUT);
    digitalWrite(newSonarTriggers[sonarIndex], HIGH);
    newSonarStates[sonarIndex] = 1;
    newSonarTimes[sonarIndex] = micros();
  }

  return false;
}

//poll the M4 (coprocessor) to read the sensor data
void loopM4() {

  // the lidars don't interfere, they all can be read at the same time  
  for(int lidarIndex=0; lidarIndex<numLidars; lidarIndex++){
    readLidar(lidarIndex);
  }

  // imu
  // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // sense.lidars[0] = ax;
  // sense.lidars[1] = ay;
  // sense.lidars[2] = az;
  // sense.lidars[3] = gx;
  // sense.sonars[0] = gy;
  // sense.sonars[1] = gz;

  // delay(100);


  // the sonars can interfere with eachother, so they can't be read at the same time
  static bool isOnNewSonar=true; //goes back and forth between old and new sonar

  static int m4SonarIndex=0;
  if(isOnNewSonar){
    if(readNewSonar(m4SonarIndex)) m4SonarIndex++;
    if(m4SonarIndex==numNewSonars) {
      m4SonarIndex=0;
      isOnNewSonar=false;
    }
  } else {
    if(readSonar(m4SonarIndex)) m4SonarIndex++;
    if(m4SonarIndex==numSonars) {
      m4SonarIndex=0;
      isOnNewSonar=true;
    }
  }
}


void readSensorData(struct sensors& data) {
  data = RPC.call("read_sensors").as<struct sensors>();
}

void printSensorData(struct sensors& data) {
  // print lidar data
  Serial.print("lid:   f ");
  Serial.print(data.lidars[0]);
  Serial.print(", \t  b ");
  Serial.print(data.lidars[1]);
  Serial.print(", \t  l ");
  Serial.print(data.lidars[2]);
  Serial.print(", \t  r ");
  Serial.print(data.lidars[3]);
  Serial.print("\t |\tson:   l ");
  Serial.print(data.sonars[0]);
  Serial.print(", \t  r ");
  Serial.print(data.sonars[1]);
  Serial.print("\t |\tnSon:   l ");
  Serial.print(data.newSonars[0]);
  Serial.print(", \t  r ");
  Serial.print(data.newSonars[1]);
  Serial.println();
}

// loop() is never called as setup() never returns
// this may need to be modified to run the state machine.
// consider usingnamespace rtos Threads as seen in previous example
void loop() {}


//interrupt function to count left encoder ticks
void LwheelSpeed() { encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed() { encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

//interrupt function for lidar

// turns off all 4 leds: red, yellow, gree, blue
void allOFF(){
  for (int l : leds)
    digitalWrite(l,LOW);
  
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  for (int l : leds) //set all leds to output
    pinMode(l,OUTPUT);
  for (int l : leds) //turn on all leds
    digitalWrite(l, HIGH);
  delay(pauseTime / 5); //wait 0.5 seconds
  for (int l : leds) //turn off all leds
    digitalWrite(l, LOW);

  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

//function prints encoder data to serial monitor
void printEncoderData() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}
  
/*
General purpose function to motor moves (motor positions and velocities)
*/
void moveMotors(long leftPosition, float leftVelocity, long rightPosition, float rightVelocity){
 
  Serial.println("move function");

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPosition);//move left wheel to relative position
  stepperRight.move(rightPosition);//move right wheel to relative position

  stepperLeft.setSpeed(leftVelocity);//set left motor speed
  stepperRight.setSpeed(rightVelocity);//set right motor speed

  //block until done
  steppers.runSpeedToPosition();
}
/*
distance (MM) to motor steps
*/
float distanceToSteps(float distance){
  //distance [mm] divided by circumference [mm] (pi times diameter, 85 mm) to steps (mult by should be 200, is 800 for some reason)
  return (distance * 800.0 / (PI*85.0));
}

//this is under the assumption that both motors are being driven, radians to motor steps
float radiansToSteps(float radians){
  //radians*trackwidth is the distance in mm that wheel needs to spin
  return distanceToSteps(radians * TRACKWIDTH / 2);

}

/*
Returns the value that is closest to 0
*/
float closestto0(float a, float b) {
  float c = a;
  float d = b;
  if(c<0) c = -c;
  if(d<0) d = -d;

  if(c<d) return a;
  return b;
}

/*
The move command moves the robot a requested linear distance and angular distance
The velocity is restricted to the slowest of the linear velocity or angular velocity
*/
void move(float lindist, float angdist, float linvel, float angvel){
  // if the signs don't agree, make them agree
  if((lindist<0)!=(linvel<0)) linvel = -linvel;
  if((angdist<0)!=(angvel<0)) angvel = -angvel;

  // if not trying to move one, set velocity to zero
  if(lindist==0) linvel=0;
  if(angdist==0) angvel=0;

  if((lindist!=0 && linvel==0) || (angdist!=0 && angvel==0)){ //If told to move at 0 veloicty, don't run
    return;
  }

  //add linear and angular distances and convert to motor steps
  float steps1 = distanceToSteps(lindist) - radiansToSteps(angdist);
  float steps2 = distanceToSteps(lindist) + radiansToSteps(angdist);
  //Slow down the faster move so they (linear and rotational moves) finish at the same time
  //skip velocity scaling if either distance is 0
  if(lindist!=0 && angdist!=0){ //If one speed is 0 don't scale to avoid divide by 0
    linvel = closestto0(linvel, lindist / (angdist / angvel));
    angvel = closestto0(angvel, angdist / (lindist / linvel));
  }
  
  //add linear and angular speeds and convert to motor steps
  float speed1 = distanceToSteps(linvel) - radiansToSteps(angvel);
  float speed2 = distanceToSteps(linvel) + radiansToSteps(angvel);

  moveMotors(steps1, speed1, steps2, speed2);
}

// move but using the MOVE_VEL and ROT_VEL constants
void move(float lindist, float angdist) {
  move(lindist, angdist, MOVE_VEL, ROT_VEL);
} 


void updateMotors() {
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}

// nonblocking, sets speeds, need to call updateMotors() rapeatedly after
// doesn't limit the speeds, so be careful of moving too fast
void moveVelo(float linvel, float angvel){

  // digitalWrite(stepperEnable, linvel==0 && angvel==0);//turns off the stepper motor driver to stop the terrible whining noise when not trying to move

  //add linear and angular distances and convert to motor steps
  // float steps1 = distanceToSteps(lindist) - radiansToSteps(angdist);
  // float steps2 = distanceToSteps(lindist) + radiansToSteps(angdist);
  //Slow down the faster move so they (linear and rotational moves) finish at the same time
  //skip velocity scaling if either distance is 0
  // if(lindist!=0 && angdist!=0){ //If one speed is 0 don't scale to avoid divide by 0
  //   linvel = closestto0(linvel, lindist / (angdist / angvel));
  //   angvel = closestto0(angvel, angdist / (lindist / linvel));
  // }
  
  //add linear and angular speeds and convert to motor steps
  float speed1 = distanceToSteps(linvel) - radiansToSteps(angvel);
  float speed2 = distanceToSteps(linvel) + radiansToSteps(angvel);

  // moveMotors(steps1, speed1, steps2, speed2);
  stepperLeft.setSpeed(speed1);//set left motor speed
  stepperRight.setSpeed(speed2);//set right motor speed
  

}

/*
  Pivots around one wheel.
  Wheel that spins is determined by the turn amount's sign and if it is to goForward

  Linear distance is set to the half the track width times the arc length
*/
void pivot(float turnRadians, bool goForward) {
  if(goForward==turnRadians>0)
    move(turnRadians*TRACKWIDTH/2, turnRadians);  
  else
    move(-turnRadians*TRACKWIDTH/2, turnRadians);  
}

/*
  Pivots around one wheel.
  Wheel that spins is determined by the turn amount's sign, always prefers going forwards
*/
void pivot(float turnRadians) {
  pivot(turnRadians, true);
}

/*
  Spins in place, turnRadians amount.
  Both mostors spin, opposite directions. No linear distance is traveled
  Positive is counterclockwise
  Blocks until motors are done moving.
*/
void spin(float turnRadians) {
  move(0, turnRadians);
}

/*
  Drive along a circle.
  circleRadius positive means go around a circle forward, negative is backwards
  turnRadians positive means turning left, negative is right

  Linear distance is set to the circle radius times the arc length
*/
void turn(float turnRadians, float circleRadius) {
  float dist = circleRadius*turnRadians;
  if(dist<0) dist = -dist;
  move(dist, turnRadians);
}

/*
  Moves the robot forward, distanceMM milimeters.
  Blocks until motors are done moving.
*/
void forward(float distanceMM) {
  move(distanceMM, 0);
}

/*
  Moves the robot backward, distanceMM milimeters.
  Blocks until motors are done moving.
*/
void reverse(float distanceMM) {
  forward(-distanceMM);
}

/*
  Stops the movement of the robot.
*/
void stop() {
  moveMotors(0,0,0,0);
}


/*
  Turns around a full circle.
  Positive diam is going around left, negative is going around right
  Turns the red led on.

  All math is handled by the turn() function
*/
void moveCircle(float diam) {
  digitalWrite(redLED, HIGH);//turn on red LED
  if(diam<0)
    turn(-2*PI, diam/2);
  else
    turn(2*PI, diam/2);
  digitalWrite(redLED, LOW);//turn off red LED
}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice, left then right, to create a figure 8 with circles of the given diameter.
  Turns the red and yellow led on.
*/
void moveFigure8(float diam) {
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  moveCircle(diam);
  moveCircle(-diam);
  digitalWrite(ylwLED, LOW);//turn off yellow LED
}

/*
  Points the robot in the angle given .
  Turns on and off the green led.
*/
void goToAngle(float angleRadians){
  digitalWrite(grnLED, HIGH);//turn on green LED
  spin(angleRadians); //handles angle logic
  digitalWrite(grnLED, LOW);//turn off green LED
}



/*
  Points the position in mm.
  Pos x is forward, pos y is to the left (Dr. Berry coordinate system)
  Turns on and off the green and yellow leds.
*/
void goToGoal(float x, float y){
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  goToAngle(atan2(y, x)); //turns on green
  digitalWrite(grnLED, HIGH);//keep on green LED
  forward(hypot(y, x));
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
}

/*
  Drives a square, forward then right.
  Turns on and off the red, green and yellow leds.
*/
void square(float len) {
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//keep on green LED
  for(int i=0; i<4; i++){
    forward(len);
    delay(100);
    spin(-PI/2); //turn right
    delay(100);
  }
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//keep off green LED
}

unsigned long lastRandomWanderTime = 0;



float randFloat(float low, float high){
  return random(low*RAND_FLOAT_STEP_WIDTH, high*RAND_FLOAT_STEP_WIDTH) * 1.0f / RAND_FLOAT_STEP_WIDTH;
}


#define ROUGH_RANDOM_WANDER_FORWARD_BIAS 3            //prefer going forwards this many times as much as going backwards
#define SMOOTH_RANDOM_WANDER_FORWARD_BIAS 2            //prefer going forwards this many times as much as going backwards


void setRandomWanderLedsOn() {
  // digitalWrite(redLED, LOW);
  // digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, HIGH);
}


void setCollideLedsOn() {
  digitalWrite(redLED, HIGH);
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);
}
void setCollideLedsOff() {
  digitalWrite(redLED, LOW);
}


void setAvoidLedsOn() {
  // digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, HIGH);
  // digitalWrite(grnLED, LOW);
}
void setAvoidLedsOff() {
  // digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
  // digitalWrite(grnLED, LOW);
}


void roughRandomWander() {
  setRandomWanderLedsOn();
  if((millis() - lastRandomWanderTime) >= 2000){
    moveVelo(randFloat(-50, ROUGH_RANDOM_WANDER_FORWARD_BIAS*50), randFloat(-ROT_VEL, ROT_VEL));

    lastRandomWanderTime = millis();
  } //end if
}




float alwaysForwardRandomWanderY() {
  setRandomWanderLedsOn();
  static float y = 0;
  if((millis() - lastRandomWanderTime) >= 1000){
    y = randFloat(-MOVE_VEL, MOVE_VEL);

    lastRandomWanderTime = millis();
  }
  return y/5;
}

float alwaysForwardRandomWanderX() {
  setRandomWanderLedsOn();
  return MOVE_VEL/5; //always forward
}

#define SENSOR_HISTORY 150
int historyIndex = 0;
struct sensors history[SENSOR_HISTORY];
struct sensors maxData; //would min and max just be noise?
struct sensors minData;
//if we were just doing avg, could do it more efficiently: change avg by (new value-old value)/SENSOR_HISTORY
//but we also want to set values to NO_READ_DIST if they were all NO_READ_DIST, and skip NO_READ_DIST values if some aren't
struct sensors avgData; 
#define numSensors numLidars+numSonars+numNewSonars


void recordSensorHistory(struct sensors& data){
  history[historyIndex++] = data;
  if(historyIndex==SENSOR_HISTORY) historyIndex = 0;

  bool isAllNoRead[numSensors];
  for(int s=0; s<numSensors; s++){
    maxData.lidars[s]=0;
    minData.lidars[s]=NO_READ_DIST;
    avgData.lidars[s]=0;
    isAllNoRead[s]=true;
    for(int i=0; i<SENSOR_HISTORY; i++){
      if(history[i].lidars[s]<NO_READ_DIST){
        maxData.lidars[s] = max(maxData.lidars[s], history[i].lidars[s]);
        minData.lidars[s] = min(minData.lidars[s], history[i].lidars[s]);
        avgData.lidars[s]+=history[i].lidars[s];
        isAllNoRead[s]=false;
      }
    }
    avgData.lidars[s]/=SENSOR_HISTORY;
    if(isAllNoRead[s]){
      maxData.lidars[s]=NO_READ_DIST;
      avgData.lidars[s]=NO_READ_DIST;
      // min is already NO_READ_DIST if there were no read values
    }
  }
}

float smoothRandomWanderLinVel = 0;
float smoothRandomWanderAngVel = 0;
void smoothRandomWander(){
  setRandomWanderLedsOn();
  if((millis() - lastRandomWanderTime) >= 10){
    smoothRandomWanderLinVel+=randFloat(-20, SMOOTH_RANDOM_WANDER_FORWARD_BIAS*20);
    smoothRandomWanderAngVel+=randFloat(-0.2, 0.2);

    if(smoothRandomWanderLinVel>MOVE_VEL)smoothRandomWanderLinVel=MOVE_VEL;
    if(-smoothRandomWanderLinVel>MOVE_VEL)smoothRandomWanderLinVel=-MOVE_VEL;
    if(smoothRandomWanderAngVel>ROT_VEL)smoothRandomWanderAngVel=ROT_VEL;
    if(-smoothRandomWanderAngVel>ROT_VEL)smoothRandomWanderAngVel=-ROT_VEL;

    moveVelo(smoothRandomWanderLinVel, smoothRandomWanderAngVel);
  
    lastRandomWanderTime = millis();
  } //end if
}

#define COLLIDE_ON_THRES 5 //distance that causes collide to stop movement
#define COLLIDE_OFF_THRES 10 //distance that causes collide to resume movement
#define COLLIDE_AUTO_OFF_TIME 1000 //after a certain amount of time, turn off collide (for page 8 of the lab2pdf)
#define COLLIDE_AUTO_OUT_TIME 5000 // stay out of collide
unsigned long collideTime = 0;
bool collideSaysToStop = false;
bool collideWantsToStop = false;

// if the sensors say something is close, sets collideSaysToStop to true
// currently only using lidars because sonars don't work well for this (for some reason)
void collide(struct sensors& data){
  float m = min(data.lidars[0], min(data.lidars[1], min(data.lidars[2], data.lidars[3])));
  if(collideWantsToStop){
    collideWantsToStop=m<COLLIDE_OFF_THRES;
    if(millis()-collideTime<COLLIDE_AUTO_OFF_TIME){
      // on for an amount of time
      collideSaysToStop = true;
    } else if(millis()-collideTime<COLLIDE_AUTO_OUT_TIME) {
      // off for an amount of time
      collideSaysToStop = false;
    } else {
      // back on again? not sure what should happen if we think we are colliding the entire time
      collideSaysToStop = true;
      digitalWrite(bluLED, HIGH);
    }
  } else {
    collideWantsToStop=m<COLLIDE_ON_THRES;
    collideTime=millis();
    collideSaysToStop = false;
    digitalWrite(bluLED, LOW);
  }


  if(collideSaysToStop){
    setCollideLedsOn();
  } else {
    setCollideLedsOff();
  }
}


void printCollideInfo() {
  Serial.print(collideSaysToStop?"   is":"isn't");
  Serial.print(" colliding");
  Serial.print("\t   |\t");
}

#define RUNAWAY_LIDAR_IDEAL_DIST 40 //distance measured is subtracted from this to get force, makes small distances move the robot faster
#define RUNAWAY_LIDAR_CUTOFF_DIST 40 //further than this is ignored
#define RUNAWAY_SONAR_IDEAL_DIST 20 //distance measured is subtracted from this to get force, makes small distances move the robot faster
#define RUNAWAY_SONAR_CUTOFF_DIST 20 //further than this is ignored


// x is forward and backward
float getSensorPushX(struct sensors& data){
  float x = 0;
  if(data.lidars[0]<RUNAWAY_LIDAR_CUTOFF_DIST) x-=RUNAWAY_LIDAR_IDEAL_DIST-data.lidars[0];
  if(data.lidars[1]<RUNAWAY_LIDAR_CUTOFF_DIST) x+=RUNAWAY_LIDAR_IDEAL_DIST-data.lidars[1];
  
  // if(data.sonars[0]<RUNAWAY_SONAR_CUTOFF_DIST) x-=RUNAWAY_SONAR_IDEAL_DIST-data.sonars[0];
  // if(data.sonars[1]<RUNAWAY_SONAR_CUTOFF_DIST) x-=RUNAWAY_SONAR_IDEAL_DIST-data.sonars[1];

  return x;
}


// y is left and right
float getSensorPushY(struct sensors& data){
  float y = 0;
  if(data.lidars[2]<RUNAWAY_LIDAR_CUTOFF_DIST) y+=RUNAWAY_LIDAR_IDEAL_DIST-data.lidars[2];
  if(data.lidars[3]<RUNAWAY_LIDAR_CUTOFF_DIST) y-=RUNAWAY_LIDAR_IDEAL_DIST-data.lidars[3];
  
  // if(data.sonars[0]<RUNAWAY_SONAR_CUTOFF_DIST) y+=RUNAWAY_SONAR_IDEAL_DIST-data.sonars[0];
  // if(data.sonars[1]<RUNAWAY_SONAR_CUTOFF_DIST) y-=RUNAWAY_SONAR_IDEAL_DIST-data.sonars[1];

  return y;
}



//M7 (main processor)
void setupM7() {
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  randomSeed(analogRead(0)); //analog0 is not connected, so use noise to set the seed


  Serial.begin(baudrate);     //start serial monitor communication
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
  Serial.println("Robot starting...Put ON TEST STAND");
}

//M7 (main processor)
void loopM7() {

  struct sensors data;
  readSensorData(data); //can be problematic if the sensors struct changes. 
  // If flash bad code that makes the red on board blink red, double press RST on the board to be able to flash again.


  recordSensorHistory(data);
  // printSensorData(data);

  // Serial.println("--");
  printSensorData(avgData);
  // printSensorData(minData);
  // printSensorData(maxData);


  //  collide:
  collide(avgData); //only using lidars for collide for now
  // printCollideInfo();
  if(collideSaysToStop) {
    moveVelo(0, 0); //stop
  } else {

    float x = 0;
    float y = 0;

    // x+=alwaysForwardRandomWanderX();
    // y+=alwaysForwardRandomWanderY();

    float sensorX = getSensorPushX(avgData)*2;
    float sensorY = getSensorPushY(avgData)*2;
    x+=sensorX;
    y+=sensorY;
    if(sensorX!=0 || sensorY!=0) setAvoidLedsOn();
    else setAvoidLedsOff();

    // float angvel = x>0? -y : y;
    float angvel = -atan2(y, x);

    float ninety = PI/2;
    float ang = 4*angvel/5;
    ang = ang>ninety ? ninety : (ang<-ninety ? -ninety : ang); //clamp


    moveVelo(x*5*cos(ang), angvel);
    
  }


  

  updateMotors();
} //end loop


//setup function for both processors
void setup() {
  RPC.begin();
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    // if on M7 CPU, run M7 setup & loop
    setupM7();
    while (1) loopM7();
  } else {
    // if on M4 CPU, run M4 setup & loop
    setupM4();
    while (1) loopM4();
  }
}