/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void setMotorEnableFlag(boolean isEnabled);
boolean isMotorFault();
