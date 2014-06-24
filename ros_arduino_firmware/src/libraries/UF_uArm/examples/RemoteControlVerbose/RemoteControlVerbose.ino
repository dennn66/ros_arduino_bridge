/************************************************************************
* File Name          : RemoteControlVerbose
* Author             : Evan
* Updated            : Evladov
* Version            : V0.0.1
* Date               : 30 May, 2014
* Description        : Serial Terminal Control 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*************************************************************************/
#include <Servo.h>
#include <EEPROM.h>
#include <UF_uArm.h>



/* Include definition of serial commands */
#include "commands.h"

/* Serial port baud rate */
//#define BAUDRATE     9600
#define BAUDRATE     115200


#define NO_LIMIT_SWITCH
#define PIEZOBUZZER

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

UF_uArm uarm;           // initialize the uArm library 

int test = 0;

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
  int arm_args[5];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
   case ARM_TEST:   
    test = test == 0? 1:0;
    Serial.println("OK");
    break;
  case ARM_ALERT:
    uarm.alert(1, 20, 0);
    Serial.println("OK");
    break;
  case ARM_SET_POSITION:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       arm_args[i] = atoi(str);
       i++;
    }
    // uarm.setPosition(stretchTemp, heightTemp, rotationTemp, handRotTemp);
    uarm.setPosition(arm_args[0], arm_args[1], arm_args[2], arm_args[3]);
    Serial.println("OK");
    break;
  case ARM_HOLD:
        /* pump action, Valve Stop. */
    if(arg1 & CATCH)   uarm.gripperCatch();
    /* pump stop, Valve action.
       Note: The air relief valve can not work for a long time,
       should be less than ten minutes. */
    if(arg1 & RELEASE) uarm.gripperRelease();
    Serial.println("OK");
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}



void setup() 
{
  Serial.begin(BAUDRATE);   // start serial port
  while(!Serial);   // wait for serial port to connect. Needed for Leonardo only
  uarm.init();          // initialize the uArm position
}

void loop()
{
//  uarm.calibration();   // if corrected, you could remove it, no harm though

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
    if(test){
      motion();
      motionReturn();
    } 
  /* delay release valve, this function must be in the main loop */
  uarm.gripperDetach();  
} 


void motion()
{
  if(test)uarm.setPosition(60, 80, 0, 0);    // stretch out
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 0, 0);  // down
  if(test)delay(400);
  if(test)uarm.gripperCatch();               // catch
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 0, 0);    // up
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 35, 0);   // rotate
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 35, 0); // down
  if(test)delay(400);
  if(test)uarm.gripperRelease();             // release
  if(test)delay(100);
  if(test)uarm.setPosition(60, 80, 35, 0);   // up
  if(test)delay(400);
  if(test)uarm.setPosition(0, 80, 0, 0);      // original position
  if(test)delay(400);
  if(test)uarm.gripperDirectDetach();        // direct detach 
  if(test)delay(500);
}

void motionReturn()
{
  if(test)uarm.setPosition(60, 80, 35, 0);    // stretch out
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 35, 0);  // down
  if(test)delay(400);
  if(test)uarm.gripperCatch();                // catch
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 35, 0);    // up
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 0, 0);     // rotate
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 0, 0);   // down
  if(test)delay(400);
  if(test)uarm.gripperRelease();              // release
  if(test)delay(100);
  if(test)uarm.setPosition(60, 80, 0, 0);     // up
  if(test)delay(400);
  if(test)uarm.setPosition(0, 80, 0, 0);       // original position
  if(test)delay(400);
  if(test)uarm.gripperDirectDetach();         // direct detach 
  if(test)delay(500);
}
