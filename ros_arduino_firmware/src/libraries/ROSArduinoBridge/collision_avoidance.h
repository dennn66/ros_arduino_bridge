/*
* Collision avoidance routines
* For use without ROS
*/


#ifdef COLLISION_AVOIDANCE

/*############################ Variable Declarations ###########################################*/
const boolean ENABLE_DEBUG=false;

/******** Move variables ***************/
const int TURN_SPEED=50;  //in ticks per frame
const int FORWARD_SPEED=50;  
const int MAX_SPEED=170;   //about 0.3ms (1 tick is 0.0018m
byte deltaSpeed=0; //increments as we continue going forward, speeding up from FORWARD_SPEED to MAX_SPEED

const int FORWARD_DELAY=100; //time to move forward without doing any check
const int STOP_DELAY=1000;
const int DELAY_TURN=1000;  //time to turn - best to make this high enough to do at least a quarter turn 
const int BACKWARD_DELAY=2000;

unsigned long moveTimer=0; //keeps track of whether we need another move action

/* variables to detect whether we are stuck */
byte turnCounterBucket=0;  //leaky bucket to keep track of turns
unsigned long bucketTimer=0;
const int BUCKET_LEAK_SPEED=1000 + 2*STOP_DELAY; //leak one value every x s
const byte MAX_TURN_BUCKET_COUNT = 5;

unsigned long stuckTimer=0;
byte stuckCounterBucket=0;
const byte MAX_STUCK_BUCKET_COUNT=3;   //number of stuck triggers before we react with escape routine
const byte STOP_MAX_STUCK_BUCKET_COUNT = 10;  //number of stuck triggers before we give up entirely
const int STUCK_CHECK_DELAY=500; // how long to wait between "stuck" checks
const byte STUCK_PWM=100; //PWM value that we consider as "stall speed" - needs to be chosen in careful combination with FORWARD_SPEED. 
//For example, following works well:
//FORWARD_SPEED=100 + STUCK_PWM=150
//FORWARD_SPEED=50 + STUCK_PWM=100

byte escapeMode=0;

/*########################### Routines ############################*/

void motorCommand(int motor1, int motor2){
  if (motor1 == 0 && motor2 == 0) {
    setMotorSpeeds(0, 0);
    moving = 0;
  } else moving = 1;
    leftPID.targetTicksPerFrame = motor1;
    rightPID.targetTicksPerFrame = motor2;
}

/**************** MOVE procedures ***********************/

void stop(){
  if (ENABLE_DEBUG) Serial.println("S");
  motorCommand(0,0);
  moveTimer = millis() + STOP_DELAY;
}

void turn(boolean isLeft){
  if (isLeft){
      //turn left
      if (ENABLE_DEBUG) Serial.println("L");
      motorCommand(-TURN_SPEED, TURN_SPEED);
  }else{
      //turn right
      if (ENABLE_DEBUG) Serial.println("R");
      motorCommand(TURN_SPEED, -TURN_SPEED);
  }
  
  turnCounterBucket++;
  moveTimer = millis() + DELAY_TURN;
 
}

void turnRight(){
  turn(false);
}

void turnLeft(){
  turn(true);
}

void moveForward(){
  if (ENABLE_DEBUG) Serial.println("F");

  motorCommand(FORWARD_SPEED+deltaSpeed, FORWARD_SPEED+deltaSpeed);
  
  //leak bucket
  if (turnCounterBucket > 0 && millis() >= bucketTimer){
    bucketTimer = millis() + BUCKET_LEAK_SPEED;
    turnCounterBucket--;
  }
  
  moveTimer = millis()+FORWARD_DELAY;

}

void moveBackward(){
  if (ENABLE_DEBUG) Serial.println("B");
  motorCommand(-FORWARD_SPEED, -FORWARD_SPEED); //re-using defined forward speeds here
  
  moveTimer = millis()+BACKWARD_DELAY;
}

void escapeTurn(){
    //do about 180 degree turn
    motorCommand(TURN_SPEED, -TURN_SPEED);
    moveTimer = millis()+DELAY_TURN*4;
    turnCounterBucket=0;
}

void turnAfterCheck(boolean doStop){
  //if moving and stop requested, stop and return
  if (moving && doStop) {
    stop();
    return;
  }
  
  if (turnCounterBucket < MAX_TURN_BUCKET_COUNT){
    //if right < left
    if (frontDistance[2].getMedian() < frontDistance[0].getMedian()) 
      turnLeft();
    else 
      turnRight();
  } else {
    escapeTurn();
  }
    
}

//Check whether we got stuck - is the robot actually moving?
boolean isStuck(){
  
  if (!moving) return false; //nothing to do 
  
  //if we've been stuck for too long time, just give up
  if (stuckCounterBucket > STOP_MAX_STUCK_BUCKET_COUNT){
    escapeMode=10; //give up
    if (ENABLE_DEBUG) Serial.println("ES");
    return true;
  }
  
  /*stuck detection routine based on encoder differences; didnt work very well */
  /*if (abs(readEncoder(0) - leftPID.prevEnc) < MIN_TICKS || abs(readEncoder(1) - rightPID.prevEnc) < MIN_TICKS){
    stuckCounterBucket++;
    if (ENABLE_DEBUG) Serial.println("EI");
  } else if (stuckCounterBucket > 0) {
    stuckCounterBucket--; //decrement bucket
    if (ENABLE_DEBUG) Serial.println("ED");
  }*/
  
  /*stuck detection routine based on PWM */
  /*adding deltaSpeed to avoid triggering it because of too high deltaspeed */
  if (leftPID.output > (STUCK_PWM+deltaSpeed) || rightPID.output > (STUCK_PWM+deltaSpeed)){
    stuckCounterBucket+=2;
    if (ENABLE_DEBUG) Serial.println("EI");
  } else if (stuckCounterBucket > 0) {
    stuckCounterBucket--; //decrement bucket
    if (ENABLE_DEBUG) Serial.println("ED");
  }
  
  if (ENABLE_DEBUG) {
    Serial.print("EC ");
    Serial.println(stuckCounterBucket);
  }
  
  if (stuckCounterBucket > MAX_STUCK_BUCKET_COUNT) {
    //reset bucket
    return true;  
  } else 
     //stuck!
    return false;
}

/* Escape routine */
void escape(){
  if (ENABLE_DEBUG) Serial.println("E");
  
  if (escapeMode != 10) escapeMode++;  //increment escapemode, protecting special give up mode
  
  switch(escapeMode){
    case 1:
        stop();
        break;
    case 2:
        moveBackward();
        break;
    case 3:
        stop();
        break;
    case 4:
        escapeTurn();
        break;
    case 5:
        stop();
        break;
    case 10:  //give up mode; just stop
        stop();
        break;
    default:
        escapeMode=0; //reset
  }
}

/* Move routine */
void move(){
 if (escapeMode>0){
       deltaSpeed=0;
       //continue escape routines
       escape();
 } else if (getMinFrontDistance() < MIN_FRONT_DIST) {
     deltaSpeed=0;
     turnAfterCheck(true);  //stop and turn
 } else { 
     //move forward
     if (getMinFrontDistance() < SLOW_DIST){
       if (deltaSpeed>=5) deltaSpeed-=5; //slow down - make sure not to go under zero!!
       else deltaSpeed=0;
     }else{
       if ((deltaSpeed + FORWARD_SPEED) < MAX_SPEED) deltaSpeed++; //speed up
     }
     moveForward();  //move forward
 }
}


#endif
