/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
#define __INA1 6
#define __INB1 8
#define __EN1DIAG1 7
#define __CS1 A0
#define __INA2 12
#define __INB2 4
#define __EN2DIAG2 7
#define __CS2 A1


#define __M1DIR 24 
#define __M1PWM 9
#define __M1FB A0
#define __M2DIR 25
#define __M2PWM 10
#define __M2FB A1
#define __nD1 26
#define __nD2 27
#define __nSF 12


void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void setMotorEnableFlag(boolean isEnabled);
boolean isMotorFault();

#if defined ARDUINO_NON_QUADRO_ENC_COUNTER
  int getMotorDirection(int i);
#endif
