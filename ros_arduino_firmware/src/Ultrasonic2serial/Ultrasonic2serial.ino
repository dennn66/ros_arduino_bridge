//
// Ultrasonic HR-SC04 sensor test
//
// http://robocraft.ru
//

#include "Ultrasonic.h"

// sensor connected to:
// Trig - 12, Echo - 13
Ultrasonic ultrasonic(4, 12);

void setup()
{
  Serial1.begin(115200); 						// start the serial port
}

void loop()
{
  float dist_cm = ultrasonic.Ranging(CM); 	// get distance
  Serial1.println(dist_cm); 					// print the distance
  
  delay(100); 								// arbitary wait time.
}
