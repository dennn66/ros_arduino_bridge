/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen
    
    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Extended by Kristof Robot with:
    - DEBUG routines (incl. free ram detection, and logic analyzer debug pins)
    - WATCHDOG timer
    - Motorfault detection and motor coasting stop
    - Additional wheel encoder counter support:
        - Onboard wheel encoder counters
        - TinyQed wheel encoder counters
    - Two types of PID controllers (position and velocity)
    - Collision avoidance routine (to be run without ROS, and three sonars)
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//#define DEBUG
#undef DEBUG

//#define COLLISION_AVOIDANCE
#undef COLLISION_AVOIDANCE

#define WATCHDOG
//#undef WATCHDOG

#ifdef WATCHDOG
  #include <avr/wdt.h>
#endif

#undef PIEZOBUZZER

#ifdef PIEZOBUZZER
#define BUZZER                  44     //
#define BUZZER_GND              42     //

void alert(int _times, int _runTime, int _stopTime)
{
    for(int _ct=0; _ct < _times; _ct++)
    {
      delay(_stopTime);
      analogWrite(BUZZER, 20);      // Almost any value can be used except 0 and 255
      delay(_runTime);
      analogWrite(BUZZER, 0);       // 0 turns it off
    }
}
#endif

#define CHECKPOWER

#ifdef CHECKPOWER
#define ROBOTVOLTAGE            A0       //

// Convert the rate into an interval 
const int CHECKPOWER_INTERVAL = 30000;
//Track the next time we make a PID calculation
unsigned long nextCHECKPOWER = CHECKPOWER_INTERVAL;
#endif


#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   #define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* TinyQED encoder counters */
   //#define TINYQED
   
   /* Encoders directly attached to Arduino board */
//   #define ARDUINO_ENC_COUNTER

   /* non-quadro Encoders directly attached to Arduino board */
   #define ARDUINO_NON_QUADRO_ENC_COUNTER
   
   #define POSITION_PID
   //#define VELOCITY_PID
   
   #define MAGNETO_HMC5883L

#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
//#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600
//#define BAUDRATE     115200

/* Maximum PWM signal */
#define MAX_PWM        400
#define MIN_PWM        100    //lowest PWM before motors start moving reliably



#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Arduino Leonardo Serial1 */
//#define Serial Serial1

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"

#define SERVOL_MIN 951 
#define SERVOL_MAX 2415 
#define SERVOR_MIN 700 
#define SERVOR_MAX 1800 
#define SERVOROT_MIN 600 
#define SERVOROT_MAX 2320 
#define SERVOHAND_OPEN_MIN 890 
#define SERVOHAND_CLOSE_MAX 1304 

#define D150A_SERVO_MIN_PUL     535
#define D150A_SERVO_MAX_PUL     2415
#define D009A_SERVO_MIN_PUL     700
#define D009A_SERVO_MAX_PUL     2650

#define SERVO_L             0    //
#define SERVO_R             1    //
#define SERVO_ROT           2    //
#define SERVO_HAND_ROT      3    //
#define SERVO_HAND          4     //
#define SERVO_CAMERA_ROT    5    //
#define SERVO_CAMERA_TILT   6     //

void setPositionMS(int _servoNum, int _position){
  int positionMS;
     if(servos[_servoNum].attached()){  
	switch(_servoNum)
	{
		case SERVO_L:
			_position = constrain(_position, SERVOL_MIN,   SERVOL_MAX);
			break;
		case SERVO_R:
			_position = constrain(_position, SERVOR_MIN,   SERVOR_MAX);
			break;
		case SERVO_ROT:
			_position = constrain(_position, SERVOROT_MIN,   SERVOROT_MAX);
			break;
		case SERVO_HAND_ROT:
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		case SERVO_HAND:
			_position = constrain(_position, SERVOHAND_OPEN_MIN,   SERVOHAND_CLOSE_MAX);
			break;
		case SERVO_CAMERA_ROT:
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		case SERVO_CAMERA_TILT:
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		default: 
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
	}
	servos[_servoNum].writeMicroseconds(_position);
     }
}    // 

void setPositionDegree(int _servoNum, int _position){
  int positionMS = 1500;
	switch(_servoNum)
	{
		case SERVO_L:
		case SERVO_R:
		case SERVO_ROT:
			positionMS = map(_position, 0, 180, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
			break;
		case SERVO_HAND_ROT:
		case SERVO_HAND:
		case SERVO_CAMERA_ROT:
		case SERVO_CAMERA_TILT:
                        positionMS = map(_position, 0, 180, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL); 
			break;
		default:  
			break;
	}
	setPositionMS( _servoNum, positionMS);
}    // 

void servoAttach(int _servoNum, int state){
  if(state == 1){
	switch(_servoNum)
	{
		case SERVO_L:
		case SERVO_R:
		case SERVO_ROT:
		        servos[_servoNum].attach(servoPins[_servoNum], D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
			break;
		case SERVO_HAND_ROT:
		case SERVO_HAND:
		case SERVO_CAMERA_ROT:
		case SERVO_CAMERA_TILT:
			servos[_servoNum].attach(servoPins[_servoNum], D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		default:  
			servos[_servoNum].attach(servoPins[_servoNum], D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
	}
  } else if(state == 0) {
        servos[_servoNum].detach();    
  }
}    //  

float getPositionRAD(int _servoNum){
  int positionMS;
        positionMS = servos[_servoNum].readMicroseconds();
	switch(_servoNum)
	{
		case SERVO_L:
		case SERVO_R:
		case SERVO_ROT:
			return map(servos[_servoNum].readMicroseconds(),D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL, -PI/2, PI/2);
			break;
		case SERVO_HAND_ROT:
		case SERVO_HAND:
		case SERVO_CAMERA_ROT:
		case SERVO_CAMERA_TILT:
			return map(servos[_servoNum].readMicroseconds(), D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL, -PI/2, PI/2);
			break;
		default:  
			return 0;
                        break;
	}
}

#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"
  
  /* Initial PID Parameters */
  const int INIT_KP = 1;    
  const int INIT_KD = 0;
  const int INIT_KI = 30;      
  const int INIT_KO = 5;  

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;
  
  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
  
  boolean isMotorDisabled=false;
#endif

#ifdef COLLISION_AVOIDANCE
  #include <NewPing.h> //IMPORTANT: assuming adapted NewPing library, which returns MAX DISTANCE when no distance detected, rather than 0!!
  #include "FastRunningMedian.h"
  #include "collision_avoidance.h"
#endif

#ifndef COLLISION_AVOIDANCE 
/* Variable initialization */
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    setPositionDegree(arg1, arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].read());
    break;
  case ALL_SERVO_READ:
    for (i = 0; i < N_SERVOS; i++) {
       Serial.print(servos[i].read());
       Serial.print(" ");
    }
    Serial.println("");
    break;
  case ALL_SERVO_STATE:
    for (i = 0; i < N_SERVOS; i++) {
       Serial.print(servos[i].attached());
       Serial.print(" ");
    }
    Serial.println("");
    break;
  case ALL_SERVO_WRITE:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       if(strlen(str) >0) setPositionDegree(i, atoi(str));
       i++;
    }
    Serial.println("OK");
    break;

    for (i = 0; i < N_SERVOS; i++) {
       Serial.print(servos[i].attached());
       Serial.print(" ");
    }
    Serial.println("");
    break;
  case SERVO_ATTACH:
    servoAttach(arg1, arg2);
    Serial.println("OK");
    break;
  case ALL_SERVO_ATTACH:
    //int i;
    for (i = 0; i < N_SERVOS; i++) {
       servoAttach(i, arg1);
    }
    Serial.println("OK");
    break;

#endif
    
#ifdef USE_BASE

#ifdef MAGNETO_HMC5883L
  case MAGNETO_READ:
    Serial.print(readMilliGauss_OnThe_XAxis());
    Serial.print(" ");
    Serial.print(readMilliGauss_OnThe_YAxis());
    Serial.print(" ");
    Serial.println(readMilliGauss_OnThe_ZAxis());
    break;
#endif
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID(); //also need to reset PID
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (isMotorDisabled) {
        setMotorEnableFlag(true);
        isMotorDisabled=false;
    }
    
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else moving = 1;
    leftPID.targetTicksPerFrame = arg1;
    rightPID.targetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    setPIDParams(pid_args[0], pid_args[1], pid_args[2], pid_args[3], PID_RATE);
    Serial.println("OK");
    break;
  case SEND_PWM:
    { //need brackets to restrict scope of newly created variables to this case statement
      lastMotorCommand =  millis();
      
      if (isMotorDisabled) {
        setMotorEnableFlag(true);
        isMotorDisabled=false;
      }
     
      int leftSpeed = arg1;
      int rightSpeed = arg2;
     
      //ensure speeds are below MAX_PWM speed
      if (leftSpeed > MAX_PWM) leftSpeed = MAX_PWM;
      else if (leftSpeed < -MAX_PWM) leftSpeed = -MAX_PWM;
      if (rightSpeed > MAX_PWM) rightSpeed = MAX_PWM;
      else if (rightSpeed < -MAX_PWM) rightSpeed = -MAX_PWM;
      
      setMotorSpeeds(leftSpeed, rightSpeed);
      moving = 0; //no need to do PID
      Serial.println("OK");
      break;
    }
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}
#endif




/* Setup function--runs once at startup. */
void setup() {
  
  Serial.begin(BAUDRATE);
//  Serial1.begin(BAUDRATE);
#ifdef PIEZOBUZZER
  pinMode(BUZZER,   OUTPUT); digitalWrite(BUZZER,   LOW);
  pinMode(BUZZER_GND,   OUTPUT); digitalWrite(BUZZER_GND,   LOW);
  alert(1, 200, 0);
#endif

  
#ifdef DEBUG
  Serial1.println("Starting up...");
  pinMode(40, OUTPUT); //cpu measurement pin for logic analyzer
  pinMode(41, OUTPUT); //PID frequency measurement pin for logic analyzer
  pinMode(42, OUTPUT); //left encoder
  pinMode(43, OUTPUT); //right encoder

  Serial1.println("CPU and PID frequency measurement PINs active.");
#endif

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef TINYQED
    //set fast I2C bus speed for TQED encoder counters
    TWBR = ((16000000L / 400000L) - 16) / 2;
  #endif
  
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B); // tell pin change mask to listen to left encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B); // tell pin change mask to listen to right encoder pins
    
    PCICR |= (1 << PCIE1) | (1 << PCIE2);   // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  #endif

  #ifdef ARDUINO_NON_QUADRO_ENC_COUNTER
    attachInterrupt(0, rightEncoderTick, RISING); //pin 2
    attachInterrupt(1, leftEncoderTick, RISING); //pin 3
  #endif
  
  #ifdef MAGNETO_HMC5883L
  initMC5883L();
  #endif
  
  initMotorController();
  //init PID
  setPIDParams(INIT_KP, INIT_KD, INIT_KI, INIT_KO, PID_RATE);
  resetPID();

  #ifdef COLLISION_AVOIDANCE
    //get first distances
    for (int i=0; i<FRONT_PING_NUM*PING_MEDIAN_NUM; i++){
      getNextDistance();
      delay(10); //need to wait at least 30ms between pings; with three sonars; 10ms
    }
    
    if (ENABLE_DEBUG) printDistances();
    
    bucketTimer=millis();
    
    if (ENABLE_DEBUG) Serial1.println("Collision Setup Ready");
  #endif
  
#endif

/* Attach servos if used */
#ifdef USE_SERVOS

  for (int i = 0; i < N_SERVOS; i++) servoAttach(i,1);  
  for (int i = 0; i < N_SERVOS; i++) setPositionDegree(i, 90);
  delay(1500);
  setPositionDegree(0, 170);
  setPositionDegree(1, 45);
  delay(1500);  
  for (int i = 0; i < N_SERVOS; i++) servoAttach(i,0);
  
#endif

#ifdef WATCHDOG
  wdt_reset();
  wdt_enable(WDTO_1S);  //reset after 1s of inactivity
#endif

#ifdef DEBUG
    Serial1.print("Free Mem:");
    Serial1.println(freeRam());
#endif

}


/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
#ifdef WATCHDOG
    //watchdog protection; if we dont reset this every X ms
    //arduino will reset
    wdt_reset();
#endif

#ifndef COLLISION_AVOIDANCE 
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
#endif
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    nextPID = millis() + PID_INTERVAL;
  #ifdef DEBUG
     digitalWrite(41, HIGH - digitalRead(41));
//    PINC = (1<<PC3);   //toggle for PID interval measurement with logic analyzer, pin A3
  #endif
    updatePID();
  }
  
  #ifdef MAGNETO_HMC5883L
  tickMC5883L();
  #endif
  
  #ifndef COLLISION_AVOIDANCE
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    //setMotorSpeeds(0, 0);
    //coast to a stop
    setMotorEnableFlag(false);
    isMotorDisabled=true;
    moving = 0;
  }
  #endif
  
#ifdef CHECKPOWER
  if (millis() > nextCHECKPOWER) {
#ifdef PIEZOBUZZER
  pinMode(BUZZER,   OUTPUT); digitalWrite(BUZZER,   LOW);
  pinMode(BUZZER_GND,   OUTPUT); digitalWrite(BUZZER_GND,   LOW);
  analogWrite(BUZZER, 20);      // Almost any value can be used except 0 and 255
  wdt_reset();
  delay(300);
  wdt_reset();
  analogWrite(BUZZER, 0);       // 0 turns it off
#endif
 
    nextCHECKPOWER = millis() + CHECKPOWER_INTERVAL;
    float rv;
    rv = analogRead(ROBOTVOLTAGE)*0.0248;
    Serial.println(rv);
    if(rv < 15.00){
      Serial.println("LOW");
      servos[7].attach(servoPins[7], D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
      servos[7].write(90);
    }
  }
#endif
  
  //detect motor faults
  //if motor is disabled, motor fault is active; so excluding that case
  if (!isMotorDisabled && isMotorFault())
  {
    Serial.println("FATAL ERROR: Motor fault - stopping");
    setMotorEnableFlag(false);
    isMotorDisabled=true;
    moving = 0;
 #ifdef WATCHDOG
    wdt_disable();
 #endif
    while(1){
      Serial.println("FATAL ERROR: Motor fault - stopped");
      delay(1000);
    }
  }
  
  #ifdef COLLISION_AVOIDANCE
      //if pingSpeed time has passed, trigger new sonar ping
      if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
        //recalibrate pingTimer - to avoid problems
        pingTimer = millis()+pingSpeed;
        getNextDistance();
      }
     
     //check whether we are stuck
     if (millis() >= stuckTimer){
         stuckTimer += STUCK_CHECK_DELAY;
         if (isStuck()) escape(); //if we are supposed to be moving, but not moving, initiate escape procedures
     }
    
    if (millis() >= moveTimer){
        //time for new movement
        if (ENABLE_DEBUG) printDistances();
        move();
    }
     
  #endif
  
  #ifdef DEBUG
    //toggle cpu measurement for logic analyzer on pin 5
    //PINC = (1<<PC2); //pin A2
    digitalWrite(40, HIGH - digitalRead(40));
  #endif

#endif
}

#ifdef DEBUG
/* Report free ram */
/* From http://jeelabs.org/2011/05/22/atmega-memory-use/ */
int freeRam () {

  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 

}
#endif



