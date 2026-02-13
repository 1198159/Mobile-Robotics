/*
  Braitenberg-Final.ino
  Ethan Harden
  Alex Stedman
  Alex Yim
  2/3/26

  This lab is about following walls and hallways, and driving to a goal avoiding obstacles.

  Key functions:
  TODO: write key functions with short descriptions



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


#include "RPC.h" //for other core

// wifi
#include <WiFi.h>
#include <SPI.h>


#include <queue>


// wifi
WiFiServer server(23);


//include all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include "wallFollow.h"
#include "goToAngleController.h"


// imu
// #include <MPU6050.h> 
#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "MPU6050_6Axis_MotionApps20.h"



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
#define max_speed 600 //maximum stepper motor speed. Prevent skipping ticks.
#define max_accel 1000 //maximum motor acceleration

int pauseTime = 2500;   //time before robot moves in ms

#define TRACKWIDTH 216   //distance between the wheels in mm
#define MOVE_VEL 50     //velocity of movement, mm/s
#define ROT_VEL 2   //velocity of movement, rad/s


#define betterGoToGoalLinThresh 100
#define betterGoToGoalAngThresh 0.1

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset


// lidar pins
#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 11

// old sonar pins
#define leftSnr 2
#define rightSnr 3

// new sonar pins
#define purpleSnrTrig 31
#define purpleSnrEcho 33
#define orangeSnrTrig 30
#define orangeSnrEcho 32

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

#define RAND_FLOAT_STEP_WIDTH 100     //makes it so you get random to 0.01 place
#define RANDOM_WANDER_TIME 1000   //how long random wander waits between generating new targets
unsigned long lastRandomWanderTime = 0;
#define RANDOM_WANDER_SCALE 0.2 //how much force random wander causes

#define LOOP_TIME 10 //how long the main processor waits between loops
unsigned long lastLoopTime = 0;

#define TIMES_NO_READ_THRES 10 // how many times a sensor needs to not read in order to set the distance to far away

#define NO_READ_DIST 1000 // how far is far away when a sensor doesn't read

#define COLLIDE_ON_THRES 5 //distance that causes collide to stop movement
#define COLLIDE_OFF_THRES 10 //distance that causes collide to resume movement
#define COLLIDE_AUTO_OFF_TIME 1000 //after a certain amount of time, turn off collide (for page 8 of the lab2pdf)
#define COLLIDE_AUTO_OUT_TIME 5000 // stay out of collide
unsigned long collideTime = 0;
bool collideSaysToStop = false;
bool collideWantsToStop = false;

#define SENSOR_PUSH_LIDAR_IDEAL_DIST 60 //distance measured is subtracted from this to get force, makes small distances move the robot faster
#define SENSOR_PUSH_LIDAR_CUTOFF_DIST 60 //further than this is ignored
#define SENSOR_PUSH_SONAR_IDEAL_DIST 40 //distance measured is subtracted from this to get force, makes small distances move the robot faster
#define SENSOR_PUSH_SONAR_CUTOFF_DIST 40 //further than this is ignored
#define followDist 12.5 //follow tries to follow at this distance
#define SENSOR_PUSH_SCALE 6 //how much force sensor push causes

#define AVOID_SPECIAL_CASE_THRESH 20
#define NINETY_DEGREES PI/2
#define AVOID_SPECIAL_CASE_ANGLE_THRESH NINETY_DEGREES/6
#define AVOID_LINVEL_SCALE 5
#define AVOID_SPECIAL_CASE_WAIT_TIME_LONG 200
#define AVOID_SPECIAL_CASE_WAIT_TIME_SHORT 50

#define SONAR_DIFF_SCALE 2 //for follow, impacts how fast the sonar turns the robot
#define SONAR_SUM_SCALE 25 //for follow, impacts how fast the sonar pulls to or pushes from a wall

// imu
MPU6050 accelgyro;
// int16_t ax, ay, az;
// int16_t gx, gy, gz;
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
VectorFloat gravity;    // [x, y, z]            Gravity vector
Quaternion q;           // [w, x, y, z]         Quaternion container
//float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// struct for all sensor readings
struct sensors {
  float lidars[numLidars];
  float sonars[numSonars];
  float newSonars[numNewSonars];
  float ypr[3];
  // this defines some helper functions that allow RPC to send our struct (I [Berry] found this on a random forum)
  MSGPACK_DEFINE_ARRAY(lidars, sonars, newSonars, ypr);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} sense;


struct odometry {
  float currentX;
  float currentY;
  float currentAngle; //would be just yaw
  MSGPACK_DEFINE_ARRAY(currentX, currentY, currentAngle);
} odo;


struct targetPosition {
  float x;
  float y;
  MSGPACK_DEFINE_ARRAY(x, y);
};

// co handles
std::queue<targetPosition> queuedTargets{};


bool reached = false;
// co calls
void reached_target() {
  if(queuedTargets.size()>1) {
    queuedTargets.pop(); //if there would be at least 1 thing left, remove something from the queue
  } else { //nothing left in queue
    reached = true;
  }
}

// when a sensor sucessfully reads, the corresponding sensor gets set to 0, each time it doesn't read it increments by one
struct timesNoReadStruct {
  unsigned int lidars[numLidars];
  unsigned int sonars[numSonars];
  unsigned int newSonars[numNewSonars];
  // doesn't need to be sent across processors
  // MSGPACK_DEFINE_ARRAY(lidar_front, lidar_back, lidar_left, lidar_right, sonar_left, sonar_right);
} timesNoRead;

// rpc: main calls on co
struct sensors read_sensors() {
  return sense;
}

// rpc: main calls on co
struct odometry read_odometry() {
  return odo;
}

bool is_at_position(){
  return reached;
}

// relative to the most recently added target
void add_relative_target(struct targetPosition data){
  struct targetPosition mostRecent{0, 0};
  if(!queuedTargets.empty())
    mostRecent = queuedTargets.back(); //calculate the absolute coord using the most recent one
  mostRecent.x+=data.x;
  mostRecent.y+=data.y;
  queuedTargets.push(mostRecent);
}

void add_absolute_target(struct targetPosition data){
  queuedTargets.push(data);
}

// co calls
struct targetPosition get_target_position() {
  if(queuedTargets.empty()) return {0, 0}; //no movement if no queue
  return queuedTargets.front(); //return the next thing (the one that would be removed with pop())
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

// initializes the lidar for the index given
void initLidar(int lidarIndex) {
  timesNoRead.lidars[lidarIndex] = TIMES_NO_READ_THRES;
  sense.lidars[lidarIndex] = NO_READ_DIST;
}

// initializes the (old) sonar for the index given
void initSonar(int sonarIndex) {
  timesNoRead.sonars[sonarIndex] = TIMES_NO_READ_THRES;
  sense.sonars[sonarIndex] = NO_READ_DIST;
}

// initializes the new sonar for the index given
void initNewSonar(int sonarIndex) {
  timesNoRead.newSonars[sonarIndex] = TIMES_NO_READ_THRES;
  sense.newSonars[sonarIndex] = NO_READ_DIST;

  pinMode(newSonarTriggers[sonarIndex], OUTPUT);
  digitalWrite(newSonarTriggers[sonarIndex], LOW);  

  pinMode(newSonarEchos[sonarIndex], INPUT);
}

// initializes the imu
void initImu() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  accelgyro.initialize();

  // Serial.println(F("Testing MPU6050 connection..."));
  if(accelgyro.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    // while(true);
  }
  else {
    // Serial.println("MPU6050 connection successful");

      /* Initializate and configure the DMP*/
    // Serial.println(F("Initializing DMP..."));
    devStatus = accelgyro.dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);

    /* Making sure it worked (returns 0 if so) */ 
    if (devStatus == 0) {
      accelgyro.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
      accelgyro.CalibrateGyro(6);
      // Serial.println("These are the Active offsets: ");
      // accelgyro.PrintActiveOffsets();
      // Serial.println(F("Enabling DMP..."));   //Turning ON DMP
      accelgyro.setDMPEnabled(true);

      MPUIntStatus = accelgyro.getIntStatus();

      /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
      // Serial.println(F("DMP ready! Waiting for first interrupt..."));
      DMPReady = true;
      packetSize = accelgyro.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    }

  }

  Serial.println();
}

// sets the values in the ypr array (yaw pitch roll, each in radians).
// yaw is between -pi and pi.
// from MPU6050_DMP6_ImuData_for_ROS.ino from the MPU6050 example files
void readImuYPR() {
  if (!DMPReady) return;

  if (accelgyro.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    accelgyro.dmpGetQuaternion(&q, FIFOBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(sense.ypr, &q, &gravity);
  }
}

// prints yaw, pitch, and roll, in radians
void printImuYPR() {
  Serial.print("yaw: ");
  Serial.print(sense.ypr[0]);
  Serial.print(", \t  pitch: ");
  Serial.print(sense.ypr[1]);
  Serial.print(", \t  roll: ");
  Serial.println(sense.ypr[2]);
}

void initAllSensors() {
  for(int i=0; i<numLidars; i++)
    initLidar(i);
  for(int i=0; i<numSonars; i++)
    initSonar(i);
  for(int i=0; i<numNewSonars; i++)
    initNewSonar(i);
}


// Reads the lidar given the lidar index.
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
      if(dist<LIDAR_FAR_THRESH && dist>0) {//temp: may break things. If distance read negative, say no read
        sense.lidars[lidarIndex] = dist;
        timesNoRead.lidars[lidarIndex] = 0;
      } else {
        noRead = true;
      }
    }
  } else if(lidarRisingTimes[lidarIndex]+lidarDisconnectTimeout<=micros()){ //no change for too long
    // get a strike, but don't change any pins to give up read
    noRead = true;
    lidarRisingTimes[lidarIndex] = micros(); //prevent getting all strikes too quickly
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

// Reads the (old) sonar given the sonar index.
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

// Reads the new sonar given the sonar index.
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


void readAllSensors() {
  // the lidars don't interfere, they all can be read at the same time  
  for(int lidarIndex=0; lidarIndex<numLidars; lidarIndex++){
    readLidar(lidarIndex);
  }


  // the sonars can interfere with eachother, so they can't be read at the same time
  // static bool isOnNewSonar=true; //goes back and forth between old and new sonar

  static int m4SonarIndex=0;
  // if(isOnNewSonar){
    if(readNewSonar(m4SonarIndex)) m4SonarIndex++;
    if(m4SonarIndex==numNewSonars) {
      m4SonarIndex=0;
      // isOnNewSonar=false;
    }
  // } else {
  //   if(readSonar(m4SonarIndex)) m4SonarIndex++;
  //   if(m4SonarIndex==numSonars) {
  //     m4SonarIndex=0;
  //     isOnNewSonar=true;
  //   }
  // }
}


// function for the m7 to call to get data from the m4
void readSensorData(struct sensors& data) {
  data = RPC.call("read_sensors").as<struct sensors>();
}

void readOdometryData(struct odometry& data) {
  data = RPC.call("read_odometry").as<struct odometry>();
}

void addRelativeTarget(struct targetPosition data) {
  RPC.call("add_relative_target", data);
}

void addAbsoluteTarget(struct targetPosition data) {
  RPC.call("add_absolute_target", data);
}

bool isAtPosition(){
  return RPC.call("is_at_position").as<bool>();
}

// function to print lidar, (old) sonar, and new sonar
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

// turns off all 4 leds: red, yellow, green, blue
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
  // stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  // stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

//function prints encoder data to serial monitor
void printEncoderData() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    long deltaLeft = encoder[LEFT];  
    encoder[LEFT] = 0; 
    long deltaRight = encoder[RIGHT];  
    encoder[RIGHT] = 0; 
    accumTicks[LEFT]+=deltaLeft;
    accumTicks[RIGHT]+=deltaRight;
    Serial.print("Accumulated Ticks: ");
    Serial.print("\tl: ");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tr: ");
    Serial.println(accumTicks[RIGHT]);
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

/*
motor steps to distance (MM)
*/
float stepsToDistance(float steps){
  //distance [mm] divided by circumference [mm] (pi times diameter, 85 mm) to steps (mult by should be 200, is 800 for some reason)
  return steps * (PI*85.0) / 800.0;
}

// radians to travel to motor steps
//this is under the assumption that both motors are being driven, radians to motor steps
float radiansToSteps(float radians){
  //radians*trackwidth is the distance in mm that wheel needs to spin
  return distanceToSteps(radians * TRACKWIDTH / 2);
}

// motor steps to radians travelled
float stepsToRadians(float steps) {
  return stepsToDistance(steps / TRACKWIDTH * 2);
}


long prevLeft=0;
long prevRight=0;

// uses motor ticks and imu yaw to update where the robot thinks it is
void updateOdometry() {
  odo.currentAngle = sense.ypr[0];
  
  long currentLeft = stepperLeft.currentPosition();
  long currentRight = stepperRight.currentPosition();
  float deltaLeft = currentLeft-prevLeft;
  float deltaRight = currentRight-prevRight;
  prevLeft=currentLeft;
  prevRight=currentRight;

  float deltaDist = stepsToDistance(deltaLeft+deltaRight)/2;
  // float deltaAngle = stepsToRadians(deltaRight-deltaLeft)/2;
  // odo.currentAngle+=deltaAngle;

  odo.currentX+=deltaDist*cos(odo.currentAngle);
  odo.currentY+=deltaDist*sin(odo.currentAngle);
}

// function to print where the robot thinks it is
void printOdometry() {
  static unsigned long timer = 0;
  if (millis() - timer > 100) {
    
    Serial.print("motor ticks odo: ");
    Serial.print("\tx: ");
    Serial.print(odo.currentX);
    Serial.print("\ty: ");
    Serial.print(odo.currentY);
    Serial.print("\ta: ");
    Serial.println(odo.currentAngle);
    timer = millis();
  }
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

// tells both motors to run()
void updateMotors() {
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}

// nonblocking, sets speeds, need to call updateMotors() rapeatedly after
// doesn't limit the speeds, so be careful of moving too fast
float pastSpeed1 = 0, pastSpeed2 = 0;
#define MAX_SPEED_DELTA  20.0f
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
  float speed1 = clamp(distanceToSteps(linvel) - radiansToSteps(angvel), pastSpeed1 - MAX_SPEED_DELTA, pastSpeed1 + MAX_SPEED_DELTA);
  float speed2 = clamp(distanceToSteps(linvel) + radiansToSteps(angvel), pastSpeed2 - MAX_SPEED_DELTA, pastSpeed2 + MAX_SPEED_DELTA);

  pastSpeed1 = speed1;
  pastSpeed2 = speed2;
  // moveMotors(steps1, speed1, steps2, speed2);
  stepperLeft.setSpeed(speed1);//set left motor speed
  stepperRight.setSpeed(speed2);//set right motor speed
  

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
  Moves the robot forward, distanceMM milimeters.
  Blocks until motors are done moving.
*/
void forward(float distanceMM) {
  move(distanceMM, 0);
}

/*
  Stops the movement of the robot.
*/
void stop() {
  moveMotors(0,0,0,0);
}

// gets a random float, used in random wander
float randFloat(float low, float high){
  return random(low*RAND_FLOAT_STEP_WIDTH, high*RAND_FLOAT_STEP_WIDTH) * 1.0f / RAND_FLOAT_STEP_WIDTH;
}

// sets the random wander leds (green on)
void setRandomWanderLedsOn() {
  digitalWrite(grnLED, HIGH);
}

// sets the collide leds (red on, yellow and green off)
void setCollideLedsOn() {
  digitalWrite(redLED, HIGH);
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);
}
// un-sets the collide leds (red off)
void setCollideLedsOff() {
  digitalWrite(redLED, LOW);
}

// sets the avoid leds (yellow on)
void setAvoidLedsOn() {
  digitalWrite(ylwLED, HIGH);
}
// un-sets the avoid leds (yellow off)
void setAvoidLedsOff() {
  digitalWrite(ylwLED, LOW);
}
// sets the follow leds (yellow and green on)
void setFollowLedsOn() {
  digitalWrite(ylwLED, HIGH);
  digitalWrite(grnLED, HIGH);
}
// un-sets the follow leds (yellow and green off)
void setFollowLedsOff() {
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);
}

// gets the y component of always forward random wander. This varies
float alwaysForwardRandomWanderY() {
  setRandomWanderLedsOn();
  static float y = 0;
  if((millis() - lastRandomWanderTime) >= RANDOM_WANDER_TIME){
    y = randFloat(-MOVE_VEL, MOVE_VEL);

    lastRandomWanderTime = millis();
  }
  return y*RANDOM_WANDER_SCALE;
}

// gets the x component of always forward random wander. This doesn't vary (because always forward)
float alwaysForwardRandomWanderX() {
  setRandomWanderLedsOn();
  return MOVE_VEL*RANDOM_WANDER_SCALE; //always forward
}

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
      digitalWrite(bluLED, HIGH); //but still complain
    } else {
      // back on again. Stop if we think we are colliding the entire time, it probably won't get better
      collideSaysToStop = true;
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

// prints if collide says to stop
void printCollideInfo() {
  Serial.print(collideSaysToStop?"   is":"isn't");
  Serial.print(" colliding");
  Serial.print("\t   |\t");
}

// x is forward and backward
float getSensorPushX(struct sensors& data){
  float x = 0;
  if(data.lidars[0]<SENSOR_PUSH_LIDAR_CUTOFF_DIST) x-=SENSOR_PUSH_LIDAR_IDEAL_DIST-data.lidars[0];
  if(data.lidars[1]<SENSOR_PUSH_LIDAR_CUTOFF_DIST) x+=SENSOR_PUSH_LIDAR_IDEAL_DIST-data.lidars[1];
  
  return x*SENSOR_PUSH_SCALE;
}

// y is left and right
float getSensorPushY(struct sensors& data){
  float y = 0;
  if(data.lidars[2]<SENSOR_PUSH_LIDAR_CUTOFF_DIST) y+=SENSOR_PUSH_LIDAR_IDEAL_DIST-data.lidars[2];
  if(data.lidars[3]<SENSOR_PUSH_LIDAR_CUTOFF_DIST) y-=SENSOR_PUSH_LIDAR_IDEAL_DIST-data.lidars[3];
  
  return y*SENSOR_PUSH_SCALE;
}



void tryConnect() {
  // wifi
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
  } else {
    int status = WL_IDLE_STATUS;
    int i=0;
    int numSsids = 3;
    char ssid0[] = "EEEEEE";
    char ssid1[] = "ehbox";
    char ssid2[] = "computer hotspot";
    char* ssids[] = {ssid0, ssid1, ssid2};
    char pass[] = "12345678";
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to: ");
      Serial.println(ssids[i]); //pointer math!
      // Connect to WPA/WPA2 network:
      status = WiFi.begin(ssids[i], pass);

      if(status == WL_CONNECTED) break;

      // wait 0.1 seconds for connection:
      delay(100);
      i++;
      if(i==numSsids) i=0;
    }
    Serial.print(" Connected to: ");
    Serial.println(ssids[i]);
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    server.begin();
  }
}



// takes a base x and y to go to, changes it for avoiding obstacles
void avoid(float x, float y, struct sensors& data) {
  float sensorX = getSensorPushX(data);
  float sensorY = getSensorPushY(data);
  x+=sensorX;
  y+=sensorY;
  if(sensorX!=0 || sensorY!=0) setAvoidLedsOn();
  else setAvoidLedsOff();

  float angvel = -atan2(y, x);

  float ang = angvel;
  float angClamped = angvel;
  if(ang>NINETY_DEGREES) angClamped=NINETY_DEGREES;
  if(ang<-NINETY_DEGREES) angClamped=-NINETY_DEGREES;

  if(angvel>ROT_VEL) angvel = ROT_VEL;
  if(angvel<-ROT_VEL) angvel = -ROT_VEL;

  float linvel = x*AVOID_LINVEL_SCALE*cos(angClamped);
  if(linvel>MOVE_VEL) linvel=MOVE_VEL;
  if(linvel<-MOVE_VEL) linvel=-MOVE_VEL;

  // m from collide
  float m = min(data.lidars[0], min(data.lidars[1], min(data.lidars[2], data.lidars[3])));

  // edge case detection: wall in front and back, turn 90
  if(data.lidars[0]<AVOID_SPECIAL_CASE_THRESH && data.lidars[1]<AVOID_SPECIAL_CASE_THRESH && data.lidars[2]>AVOID_SPECIAL_CASE_THRESH && data.lidars[3]>AVOID_SPECIAL_CASE_THRESH){
    spin(NINETY_DEGREES);
    delay(AVOID_SPECIAL_CASE_WAIT_TIME_LONG);
    return;
  }
  
  // edge case detection: wall left and right: forward
  if(data.lidars[2]<AVOID_SPECIAL_CASE_THRESH && data.lidars[3]<AVOID_SPECIAL_CASE_THRESH && data.lidars[1]>AVOID_SPECIAL_CASE_THRESH && data.lidars[0]>AVOID_SPECIAL_CASE_THRESH){
    forward(AVOID_SPECIAL_CASE_WAIT_TIME_SHORT);
    return;
  }

  // edge case detection: wall everywhere, give up
  if(data.lidars[0]<AVOID_SPECIAL_CASE_THRESH && data.lidars[1]<AVOID_SPECIAL_CASE_THRESH && data.lidars[2]<AVOID_SPECIAL_CASE_THRESH && data.lidars[3]<AVOID_SPECIAL_CASE_THRESH){
    stop();
    delay(AVOID_SPECIAL_CASE_WAIT_TIME_LONG);
    return;
  }

  // if it wants to turn a little or it is far away, allow movement
  if(abs(ang)<AVOID_SPECIAL_CASE_ANGLE_THRESH || data.lidars[0]>AVOID_SPECIAL_CASE_THRESH){
    moveVelo(linvel, angvel);
  } else {
    // spin in place, ignoring other stuff
    spin(ang);
  }
}

// moves around to follow a moving obstacle
void follow(float x, float y, struct sensors& data) {
  float sensorX = getSensorPushX(data);
  float sensorY = getSensorPushY(data);
  x-=sensorX;
  y+=sensorY;
  if(sensorX!=0 || sensorY!=0) setFollowLedsOn();
  else setFollowLedsOff();

  float sonarDiff = 0;
  float sonarSum = 0;
  int countSonars = 0;
  if(data.newSonars[0]<SENSOR_PUSH_SONAR_CUTOFF_DIST) {
    sonarDiff-=SENSOR_PUSH_SONAR_IDEAL_DIST-data.newSonars[0];
    sonarSum+=SENSOR_PUSH_SONAR_IDEAL_DIST-data.newSonars[0];
    countSonars++;
  }
  if(data.newSonars[1]<SENSOR_PUSH_SONAR_CUTOFF_DIST) {
    sonarDiff+=SENSOR_PUSH_SONAR_IDEAL_DIST-data.newSonars[1];
    sonarSum+=SENSOR_PUSH_SONAR_IDEAL_DIST-data.newSonars[1];
    countSonars++;
  }

  y-=sonarDiff*SONAR_DIFF_SCALE;
  

  // float angvel = x>0? -y : y;

  float linvel = (followDist-sonarSum/2)*SONAR_SUM_SCALE;
  if(countSonars<numNewSonars) linvel=0;
  if(linvel>MOVE_VEL) linvel=MOVE_VEL;
  if(linvel<-MOVE_VEL) linvel=-MOVE_VEL;

  float angvel = atan2(y, x);

  float ang = angvel;
  if(angvel>ROT_VEL) angvel = ROT_VEL;
  if(angvel<-ROT_VEL) angvel = -ROT_VEL;

  // if it wants to turn a little or it is far away, allow movement
  if(abs(ang)<AVOID_SPECIAL_CASE_ANGLE_THRESH || data.lidars[0]>AVOID_SPECIAL_CASE_THRESH){
    moveVelo(linvel, angvel);
  } else {
    // spin in place, ignoring other stuff
    spin(ang);
  }  
}

float spinCount = 0;
#define SPIN_AMT 70


// Follows (drives along) a wall. Returns true if there is a wall the robot is now following, false if there is nothing to follow.
bool wallFollow(struct sensors& data){

  bool leftReading=data.lidars[2] < 50;
  bool rightReading=data.lidars[3] < 50;
  bool backReading=data.lidars[1] < 5; //driving backwards, so if we are about to go forward into something

  float linvel=0;
  float angvel=0;

  if(spinCount > 0){
    spinCount--;
    angvel = ROT_VEL;
  } else if(spinCount < 0){
    spinCount++;
    angvel = -ROT_VEL;
  } else if(leftReading&&rightReading){
    calculate(LOOP_TIME, data.lidars[3]*10 - data.lidars[2]*10, 0, &linvel, &angvel);
    // red yellow and green
    digitalWrite(redLED, HIGH);
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, HIGH);

    // turn around
    if(backReading) spinCount = 30; 
  } else if(leftReading){
    // only left reading
    calculate(LOOP_TIME, targetDistance-data.lidars[2]*10, 0, &linvel, &angvel);
    // yellow and green
    digitalWrite(redLED, LOW);
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, HIGH);

    // turn 90
    if(backReading) spinCount = 15; 
  } else if(rightReading){
    // only right
    calculate(LOOP_TIME, data.lidars[3]*10-targetDistance, 0, &linvel, &angvel);
    // red and yellow
    digitalWrite(redLED, HIGH);
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, LOW);

    // turn 90
    if(backReading) spinCount = -15; 
  } else {

    return false;

  }

  
  // follow backwards (negative both terms)
  moveVelo(-linvel, angvel);
  return true;
}

#define wallFollowAdjustedAngleThres 1

// make the sonar state sticky
int sonarStateCount = 0;
int sticky1 = 0;
int sticky2 = 0;

// Same as wallFollow except it also considers a target to go to, and will break away from the wall (return false) if the wall isn't in the way of the target.
bool wallFollowAdjusted(struct sensors& data, float targetX, float targetY){

  // printSensorData(data);
  float deltaX = targetX-odo.currentX;
  float deltaY = targetY-odo.currentY;
  float targetAngle = atan2(deltaY, deltaX);
  float targetMagnitude = hypot(deltaX, deltaY);

  bool leftReading=data.lidars[2] < 30;
  bool rightReading=data.lidars[3] < 30;
  bool backReading=data.lidars[1] < 8 && data.lidars[1] > 2; //driving backwards, so if we are about to go forward into something


  float linvel=0;
  float angvel=0;


  if(spinCount > 0){
    spinCount--;
    angvel = ROT_VEL;
  } else if(spinCount < 0){
    spinCount++;
    angvel = -ROT_VEL;
  //handle if we abt to hit something
  } else if(backReading){

    if(leftReading && rightReading) spinCount = 2 * SPIN_AMT;
    else if (rightReading) spinCount = -SPIN_AMT;
    else spinCount = SPIN_AMT;

  }else if(leftReading&&rightReading){
   
    if(targetMagnitude < 100){

      return false; //give up, let go to angle handle it
    }
    calculate(LOOP_TIME, data.lidars[3]*10 - data.lidars[2]*10, 0, &linvel, &angvel);
  
        // red yellow and green
    digitalWrite(redLED, HIGH);
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, HIGH);
    digitalWrite(bluLED, LOW);

    // turn around
  } else 
  if(leftReading){

 
    float a = targetAngle-odo.currentAngle + PI;
    if(a<-PI) a+=PI*2;
    if(a>PI) a-=PI*2;

    // Serial.print("left: ");
    // Serial.println(a);
    
    if(a > 0 || targetMagnitude < 100) return false; //give up, let go to angle handle it

    // only left reading
    calculate(LOOP_TIME, targetDistance-data.lidars[2]*10, 0, &linvel, &angvel);
  
    // yellow and green
    digitalWrite(redLED, LOW);
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, HIGH);
    digitalWrite(bluLED, LOW);

    // turn 90

  } else if(rightReading){


    float a = targetAngle-odo.currentAngle + PI;
    if(a<-PI) a+=PI*2;
    if(a>PI) a-=PI*2;
    // Serial.print("right: ");
    // Serial.println(a);
    if(a < 0 || targetMagnitude < 100){

      return false; //give up, let go to angle handle it
    }

    // only right
    calculate(LOOP_TIME, data.lidars[3]*10-targetDistance, 0, &linvel, &angvel);
  

    // red and yellow
    digitalWrite(redLED, HIGH);
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, LOW);
    digitalWrite(bluLED, LOW);

    // turn 90
  } else {
    return false;
  
  }

  if(sonarStateCount>0) sonarStateCount--;

  moveVelo(-linvel, angvel);
  return true;
}

// Go to goal (driving forwards) taking odometry into account. goToGoal is relative coordinates, this is absolute coordinates.
// Returns true if we reached the goal, false if not.
bool betterGoToGoal(float targetX, float targetY){
  float deltaX = targetX-odo.currentX;
  float deltaY = targetY-odo.currentY;
  float targetAngle = atan2(deltaY, deltaX);
  float deltaAngle = targetAngle-odo.currentAngle;

  if(deltaAngle<-PI) deltaAngle+=2*PI;
  if(deltaAngle>PI) deltaAngle-=2*PI;

  float linvel=0;
  float angvel=0;
  float deltaDist = hypot(deltaX, deltaY);
  if(abs(deltaDist)>betterGoToGoalLinThresh) linvel=MOVE_VEL; //move if we aren't close enough

  if(deltaAngle>betterGoToGoalAngThresh) angvel=ROT_VEL;
  if(deltaAngle<-betterGoToGoalAngThresh) angvel=-ROT_VEL;

  // don't rotate if you don't plan on moving
  if(linvel==0) angvel=0;


  if(abs(deltaAngle)<AVOID_SPECIAL_CASE_ANGLE_THRESH){
    moveVelo(linvel, angvel);
  } else {
    moveVelo(0, angvel);
  }  

  return linvel==0&&angvel==0;
}

// Same as betterGoToGoal except it drives backwards to the goal.
bool backwardsBetterGoToGoal(float targetX, float targetY){
  float deltaX = targetX-odo.currentX;
  float deltaY = targetY-odo.currentY;
  float targetAngle = atan2(deltaY, deltaX)+PI;
  if(targetAngle>PI) targetAngle-=PI*2;
  float deltaAngle = targetAngle-odo.currentAngle;

  float linvel=0;
  float angvel=0;
  float deltaDist = hypot(deltaX, deltaY);
  if(abs(deltaDist)>betterGoToGoalLinThresh) linvel=-MOVE_VEL; //move if we aren't close enough

  if(deltaAngle>betterGoToGoalAngThresh) angvel=ROT_VEL;
  if(deltaAngle<-betterGoToGoalAngThresh) angvel=-ROT_VEL;

  // don't rotate if you don't plan on moving
  if(linvel==0) angvel=0;

  // Serial.print(deltaAngle);
  if(abs(deltaAngle)<AVOID_SPECIAL_CASE_ANGLE_THRESH){
    moveVelo(linvel, angvel);
  } else {
    moveVelo(0, angvel);
  }  
  // Serial.print(linvel);
  return linvel==0&&angvel==0;
}



boolean alreadyConnected = false;

WiFiClient client;




int baudrate = 115200; //serial monitor baud rate'

// M4 (coprocessor) 
void setupM4() {

  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  randomSeed(analogRead(0)); //analog0 is not connected, so use noise to set the seed


  Serial.begin(baudrate);     //start serial monitor communication
  // delay(pauseTime); //always wait 2.5 seconds before the robot moves

  Serial.println("\n");


  initAllSensors();

  // set up to be sensor server
  RPC.bind("read_sensors", read_sensors);  // bind a method to return the sensor data all at once
  RPC.bind("read_odometry", read_odometry);
  RPC.bind("add_relative_target", add_relative_target);
  RPC.bind("add_absolute_target", add_absolute_target);
  RPC.bind("is_at_position", is_at_position);

  // imu
  initImu(); //takes time

 
} //end setupM4


int moveIndex = 0;
float targetXs[] = {-457.2, -914.4, -914.4};
float targetYs[] = {0, 0, -457.2};
int numMoves = 3;

float realTargetX = 0;
float realTargetY = 0;

// M4 (coprocessor) 
void loopM4() {
  readAllSensors();
  

  if((millis() - lastLoopTime) >= LOOP_TIME){
    readImuYPR();
    updateOdometry();

    struct targetPosition target = get_target_position();

    // matrix coord frame to berry coord frame
    float targetX = -target.y;
    float targetY = target.x;

    // go to angle
    float angvel=0;
    float angvelthres = 0.05;
    float linthres = 10;
    float targetang = atan2(targetY-odo.currentY, targetX-odo.currentX);
    float toTravel = hypot(targetY-odo.currentY, targetX-odo.currentX);
    calculateGoToAngle(LOOP_TIME, targetang, sense.ypr[0], &angvel);

    if(toTravel>linthres){
      reached = false;
      if(angvel>angvelthres || angvel<-angvelthres){ //ang moving not lin moving
        moveVelo(0, angvel);
      } else { //not ang moving but is lin moving
        moveVelo(200, 0);
      }
    } else { //not linear moving
      moveVelo(0, 0);
      reached_target();
    }

    lastLoopTime = millis();
  } //end if

  updateMotors();

} //end loopM4




//M7 (main processor)
void setupM7() {

  delay(1000);
  // Serial.begin(baudrate);     //start serial monitor communication

  
  // wifi
  tryConnect(); //takes time
  
  
}


unsigned int numSendMessages = 0;

#define BUFSIZE 500

// M7 (main processor)
// relative coords (world relative, doesn't rotate with robot), abs angles
// absolute coords (world relative, doesn't rotate with robot) get queued up
// coprocessor moves motors and tells main processor when it reaches a target
void loopM7() {

  
  if(WiFi.status() == 5) {
    Serial.println("We got disconnected from the hotspot. Reconnecting");
    tryConnect();
  }

  if(!alreadyConnected)
    client = server.accept(); // not blocking

  if (client) {
    if (!alreadyConnected) {
      // clear out the input buffer:
      client.flush();
      Serial.println("We have a new client");
      alreadyConnected = true;
    }

    if(!client.connected()) {
      alreadyConnected = false;
      Serial.println("Lost the client, looking for another\n");
    }

    // read from client
    if (client.available() > 0) {
      Serial.print("Message received: '");
      char buf[BUFSIZE+1];
      char outbuf[BUFSIZE+1];
      int r = client.read((uint8_t*)(&buf), BUFSIZE);
      buf[r] = '\0';
      Serial.print(buf);
      Serial.println("'");
      char* pointer = buf;

      while(*pointer != '\0'){

        // Parse command
        if(0==strncmp(buf, "ps", 2)){ //if they send 'ps' for poll sensors
          struct sensors data;
          readSensorData(data); //can be problematic if the sensors struct changes. 
          // If flash bad code that makes the red on board blink red, double press RST on the board to be able to flash again.
          Serial.println("  poll sensors");
          
          snprintf(outbuf, BUFSIZE, "%f,%f,%f,%f,%f,%f,%f\n", data.lidars[0], data.lidars[1], data.lidars[2], data.lidars[3], data.newSonars[0], data.newSonars[1], data.ypr[0]);
        } else if (0 == strncmp(buf, "atpos", 5)){
          int atPos = isAtPosition();
          snprintf(outbuf, BUFSIZE, "%d\n", atPos); //print as a number, 0 or 1
        } else if (0 == strncmp(buf, "moveto ", 7)) {
          // Move to a relative new target. Robot deals with how to rotate there.
          // Coordinates don't rotate with the robot.
          float x, y;
          if (sscanf(buf, "moveto %f %f", &x, &y) == 2) { //if sscanf finds 2 values to fill
            addRelativeTarget({x, y});

            Serial.println("  queued moveto");
            snprintf(outbuf, BUFSIZE, "queued moveto target (%f, %f)\n", x, y);
          } else {
            snprintf(outbuf, BUFSIZE, "error: moveto didn't find 2 floats\n");
          }
        } else { //if they sent an unrecognized
          Serial.println("  unrecognized");
          snprintf(outbuf, BUFSIZE, "Unrecognized command num %d\n", numSendMessages++);
        }

        char* f = std::find(pointer, buf+BUFSIZE, '\n'); //start point, end point, what to find
        if(f==buf+BUFSIZE) break; //if didn't send \n at the end
        pointer = f+1; //move 1 past the \n
      }

      // Send response
      client.write(outbuf);
      Serial.print("Sending back: '");
      Serial.print(outbuf);
      Serial.println("'");
    }
  }
} // end loopM7



// Optional: Helper function to calculate error based on wall distances
float calculateWallDistanceError(struct sensors data) {
  // Example: Compare expected wall distances to actual sensor readings
  // Return a metric of how far off the position estimate might be
  
  float expectedFrontDist = 50.0;  // cm, example
  float actualFrontDist = data.lidars[0];
  
  float error = abs(expectedFrontDist - actualFrontDist) / 10.0;  // normalized
  return error;
}

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
