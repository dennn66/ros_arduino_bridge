/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 10;
int Kd = 3;
int Ki = 6;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(0);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(1);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (Kp * Perror - Kd * (input - p->PrevInput) + (p->ITerm + Ki * Perror)) / Ko;
  p->PrevEnc = p->Encoder;

  /*
  * Accumulate Integral error *or* Limit output.
  * 
  * Avoid motor moving back when requesting forward movement, and vice versa (avoid oscillating around 0)
  * Also avoid sending output of 0 (stopping motors)
  *
  * Stop accumulating integral error when output is limited.
  */
  if (p->TargetTicksPerFrame > 0 && output <= 0)
    output = 1;
  else if (p->TargetTicksPerFrame < 0 && output >= 0)
    output = -1;
  else if (output > MAX_PWM)
    output = MAX_PWM;
  else if (output < -MAX_PWM)
    output = -MAX_PWM;
  else
    /*
    * Allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Normalize speed to values within [-MAX_MOTOR_DRIVER_PWM, -MIN_PWM] and [MIN_PWM, MAX_MOTOR_DRIVER_PWM] */
/* Avoid speeds in interval ]-MIN_PWM, MIN_PWM[ */
int normalizeSpeed(int spd){
    int mapped_spd = spd;
    
    if (spd > 0) 
      mapped_spd = map(spd, 1, MAX_MOTOR_DRIVER_PWM, MIN_PWM, MAX_MOTOR_DRIVER_PWM);
    else if (spd < 0) 
      mapped_spd = map(spd, -MAX_MOTOR_DRIVER_PWM, -1, -MAX_MOTOR_DRIVER_PWM, -MIN_PWM);
    
    return mapped_spd;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(0);
  rightPID.Encoder = readEncoder(1);

  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * Most importantly, keep Encoder and PrevEnc synced; use that as criteria whether we need reset
    */
    if (leftPID.PrevEnc != leftPID.Encoder || rightPID.PrevEnc != rightPID.Encoder) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the normalized motor speeds accordingly */
  setMotorSpeeds(normalizeSpeed(leftPID.output), normalizeSpeed(rightPID.output));
}




