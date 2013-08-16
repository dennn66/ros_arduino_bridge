/* *************************************************************
   Encoder definitions
   
   Add a "#if defined" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#if defined ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined TINYQED
  /* Tiny QED Encoder Counter - https://github.com/renbotics/TQED */
  #include <Wire.h>
  #include <TQED.h>

  /* Create the encoder counter objects */
  TQED qedLeft(0x38);     // Change the I2C address as needed
  TQED qedRight(0x36);    // Change the I2C address as needed

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return qedLeft.getCount();
    else return qedRight.getCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      qedLeft.resetCount();
      return;
    } else { 
      qedRight.resetCount();
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

