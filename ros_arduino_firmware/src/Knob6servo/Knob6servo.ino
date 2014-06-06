// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo6;  // create servo object to control a servo 
Servo myservo12;  // create servo object to control a servo 
Servo myservo9;  // create servo object to control a servo 
Servo myservo10;  // create servo object to control a servo 
Servo myservo11;  // create servo object to control a servo 
 
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  myservo12.attach(12);  // attaches the servo on pin 9 to the servo object 
  myservo6.attach(6);  // attaches the servo on pin 9 to the servo object 
  myservo9.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo10.attach(10);  // attaches the servo on pin 9 to the servo object 
  myservo11.attach(11);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservo6.write(val);                  // sets the servo position according to the scaled value 
  myservo12.write(val);                  // sets the servo position according to the scaled value 
  myservo9.write(val);                  // sets the servo position according to the scaled value 
  myservo10.write(val);                  // sets the servo position according to the scaled value 
  myservo11.write(val);                  // sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 
} 
