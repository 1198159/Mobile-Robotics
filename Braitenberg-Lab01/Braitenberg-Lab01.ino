
/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
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

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5,6,7};      //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 49  //left stepper motor direction pin , WAS 53, pin broke, need that pin cleared

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define max_speed 1500 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 3000;   //delay for printing data

#define TRACKWIDTH 215   //distance between the wheels
#define MOVE_VEL 100     //velocity of movement, mm/s
#define ROT_VEL 1     //velocity of movement, rad/s

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset


// Helper Functions

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

void allOFF(){
  for (int i = 0;i<3;i++){
    digitalWrite(leds[i],LOW);
  }
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
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

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
void print_encoder_data() {
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
  
/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it.
*  Controls both motors at the same time, and will wait for both to finish if one finishes first.
*/
void runToStop () {
  
  bool rightStopped = false;
  bool leftStopped = false;

  while (true) {
    if (!stepperRight.run()) {
      rightStopped = true;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = true;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      return;
    }
  }
}


/*
   The move1() function will move the robot forward one full rotation and backwared on
   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1() {
  Serial.println("move1 function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
}

/*
   The move2() function will use AccelStepper library functions to move the robot
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move2() {
  Serial.println("move2 function");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  stepperRight.moveTo(800);//move one full rotation forward relative to current position
  stepperLeft.moveTo(800);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
  stepperRight.moveTo(0);//move one full rotation backward relative to current position
  stepperLeft.moveTo(0);//move one full rotation backward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
}

/*
   The move3() function will use the MultiStepper() class to move both motors at once
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move3() {
  Serial.println("move3 function");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  long positions[2]; // Array of desired stepper positions
  positions[0] = 800;//right motor absolute position
  positions[1] = 800;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
  // Move to a different coordinate
  positions[0] = 0;//right motor absolute position
  positions[1] = 0;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
}

/*this function will move to target at 2 different speeds - absolute position*/
void move4() {

  Serial.println("move4 function");
  int leftPos = 5000;//right motor absolute position
  int rightPos = 1000;//left motor absolute position
  int leftSpd = 1000;//right motor speed
  int rightSpd = 200; //left motor speed

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  stepperLeft.moveTo(leftPos);//left motor absolute position
  stepperRight.moveTo(rightPos);//right motor absolute position
  stepperLeft.setSpeed(leftSpd);
  stepperRight.setSpeed(rightSpd);
  steppers.runSpeedToPosition(); // Blocks until all are in position

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

}

/*This function will move continuously at 2 different speeds*/
void move5() {
  Serial.println("move5 function");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED

  int leftSpd = 1000;//right motor speed
  int rightSpd = 200; //left motor speed

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  while(true){
      runAtSpeed();
  }
}


/*this function will move to target at 2 different speeds - relative position*/
void move6() {

  Serial.println("move6 function");
  int leftPos = 5000;//right motor absolute position
  int rightPos = 1000;//left motor absolute position
  int leftSpd = 1000;//right motor speed
  int rightSpd = 200; //left motor speed

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
General purpose function to motor moves (motor positions and velocities)
*/
void moveMotors(long leftPosition, float leftVelocity, long rightPosition, float rightVelocity){
 
  Serial.println("move function");


  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPosition);//move left wheel to relative position
  stepperRight.move(rightPosition);//move right wheel to relative position

  stepperLeft.setSpeed(leftVelocity);//set left motor speed
  stepperRight.setSpeed(rightVelocity);//set right motor speed

  steppers.runSpeedToPosition();

  // runToStop();//run until the robot reaches the target
}
/*
MM to Steps
*/
float distanceToSteps(float distance){
  //distance [mm] divided by circumference [mm] (pi times diameter, 85 mm) to steps (mult by should be 200, is 800 for some reason)
  return (distance * 800.0 / (PI*85.0));
}
//this is under the assumption that both MOTOR IS BEING DRIVEN
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


// Move:
void move(float lindist, float angdist, float linvel, float angvel){
  // if the signs don't agree, make them agree
  if((lindist<0)!=(linvel<0)) linvel = -linvel;
  if((angdist<0)!=(angvel<0)) angvel = -angvel;

  // if not trying to move one, set velocity to zero
  if(lindist==0) linvel=0;
  if(angdist==0) angvel=0;

  if((lindist!=0 && linvel==0) || (angdist!=0 && angvel==0)){
    return;
  }

  float steps1 = distanceToSteps(lindist) - radiansToSteps(angdist);
  float steps2 = distanceToSteps(lindist) + radiansToSteps(angdist);
  //Slow down the faster move so they (linear and rotational moves) finish at the same time
  //skip velocity scaling if either distance is 0
  if(lindist!=0 && angdist!=0){
    linvel = closestto0(linvel, lindist / (angdist / angvel));
    angvel = closestto0(angvel, angdist / (lindist / linvel));
  }
  
  float speed1 = distanceToSteps(linvel) - radiansToSteps(angvel);
  float speed2 = distanceToSteps(linvel) + radiansToSteps(angvel);

  moveMotors(steps1, speed1, steps2, speed2);
}

// move but using the MOVE_VEL and ROT_VEL constants
void move(float lindist, float angdist) {
  move(lindist, angdist, MOVE_VEL, ROT_VEL);
} 

// go-to-angle(int angle, angvel)
// move(0, targetangle – currentangle, 0, angvel)

// go-to-goal(int x, int y)
// angle = atan2(y – currentY, x - currentX)
// go-to-angle(int angle, angvel)
// move (sqrt((y – currentY)^2 + (x – currentX)^2))

// moveSquare(int side)
// repeat 4x
// move(side, 0, linvel, 0)
// move(0, pi/2, 0, angvel)


// moveCircle(int diam, int dir)
// move(diam * pi, 2pi, linvel, angvel)


/*
  Pivots around one wheel.
  Wheel that spins is determined by the turn amount's sign and if it is to goForward
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
  Both mostors spin, opposite directions.
  Positive is left
  Blocks until motors are done moving.
*/
void spin(float turnRadians) {
  move(0, turnRadians);
}

/*
  Drive along a circle.
  circleRadius positive means go around a circle forward, negative is backwards
  turnRadians positive means turning left, negative is right
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
void forward(int distanceMM) {
  move(distanceMM, 0);
}

/*
  Moves the robot backward, distanceMM milimeters.
  Blocks until motors are done moving.
*/
void reverse(int distanceMM) {
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
*/
void moveCircle(float diam) {
  if(diam<0)
    turn(-2*PI, diam/2);
  else
    turn(2*PI, diam/2);
}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice, left then right, to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(float diam) {
  moveCircle(diam);
  // delay(wait_time);
  moveCircle(-diam);
}


//// MAIN
void setup()
{
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder


  Serial.begin(baudrate);     //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
}

// does the lab1 demo
void demonstration1() {
  forward(200); //forward 200mm
  delay(wait_time);
  reverse(200); //backward 200mm
  delay(wait_time);

  pivot(PI/2); //pivot forward on left wheel, 90degrees
  delay(wait_time);
  pivot(-PI/2); //pivot forward on right wheel, 90degrees
  delay(wait_time);

  turn(PI/2, 200); //turn forward, left, around a 200mm radius circle, 90degrees of the circle
  delay(wait_time);
  turn(-PI, 200); //turn forward, right, around a 200mm radius circle, 180degrees of the circle
  delay(wait_time);

  spin(PI); //spin in place left, 180degrees
  delay(wait_time);
  spin(-PI/2); //spin in place left, 90degrees
  delay(wait_time);

  moveCircle(280); //move left around a 280mm diameter circle
  delay(wait_time);

  moveFigure8(130); //move left around two 130mm diameter circles
}

void loop()
{
  //uncomment each function one at a time to see what the code does
  // move1();//call move back and forth function
  // move2();//call move back and forth function with AccelStepper library functions
  // move3();//call move back and forth function with MultiStepper library functions
  // move4(); //move to target position with 2 different speeds - absolute position
   //move5(); //move continuously with 2 different speeds
   // move6(); //move to target position with 2 different speeds - relative position
  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  //print_encoder_data();   //prints encoder data


  demonstration1();

  delay(wait_time);               //wait to move robot or read data
}