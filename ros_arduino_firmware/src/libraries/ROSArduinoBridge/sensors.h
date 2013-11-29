/* Functions for various sensor types */

float microsecondsToCm(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per cm.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long Ping(int pin) {
  long duration, range;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  // convert the time into meters
  range = microsecondsToCm(duration);
  
  return(range);
}


/******* SONAR variables *****************/
const byte MAX_DISTANCE=100; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm. No reason to wait longer for ping to return than this.
const byte SLOW_DIST=150;  //distance to use for starting to slow down
const byte MIN_FRONT_DIST=30;  //distance to stop immediately, and seek new direction

NewPing sonarFront[3] = { // Sensor object array.
  NewPing(11, 11, MAX_DISTANCE), //left
  NewPing(6, 6, MAX_DISTANCE),   //middle
  NewPing(5, 5, MAX_DISTANCE)   //right
};

const int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second; retriggering of same sonar is best kept above 30ms
unsigned long pingTimer;     // Holds the next ping time.
byte pingPointer=0;            //holds position of next sonar to use; will overflow by design
const byte FRONT_PING_NUM=3;
const byte PING_MEDIAN_NUM=3;  //holds number of pings for median filter

//int frontDistance[FRONT_PING_NUM][PING_MEDIAN_NUM]; //init distance on 0, left, center, right
FastRunningMedian<unsigned int,5, 0> frontDistance[FRONT_PING_NUM];
//int backDistance[3]= {0, 0, 0}; //init distance on 0, left, center, right

const unsigned int MAX_ECHO_TIME = min(MAX_DISTANCE, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.

/* 
* Get next distance by pinging using the next sonar in row
* and adding the value to its running median
*/
void getNextDistance(){
  byte sonarPos = pingPointer%FRONT_PING_NUM;
  
  frontDistance[sonarPos].addValue(sonarFront[sonarPos].ping_cm());
  
  pingPointer++; //increment, and let it overflow when it reaches max (255)
}

/* Return minimal front distance detected */
int getMedianPing(byte sonarPos){
   return frontDistance[sonarPos].getMedian();
}

/* Return minimal front distance detected */
int getMinFrontDistance(){
   return min(frontDistance[2].getMedian(),min(frontDistance[0].getMedian(),frontDistance[1].getMedian()));
}

void printDistances(){
  String space=" ";
  Serial.print(frontDistance[0].getMedian());
  Serial.print(space);
  Serial.print(frontDistance[1].getMedian());
  Serial.print(space);
  Serial.println(frontDistance[2].getMedian());
}
