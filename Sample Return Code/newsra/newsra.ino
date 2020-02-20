#include <Wire.h>

/*
 *
 *
 * IN DEVELOPMENT, THIS WILL NOT WORK WHATSOEVER
 * General layout for continuing further
 *
 * -EMBEDDED TEAM
 */


// Drill Motor (Use OUT1 and OUT2 on  MotorDriver "A")
#define InputA_Drill 30
#define InputB_Drill 31

// Elevator Motor (Use OUT3 and OUT4 on  MotorDriver "A")
#define InputA_Elevator 32
#define InputB_Elevator 33

// Bin Gear Motor (Use OUT1 and OUT2 on  MotorDriver "B")
#define InputA_Bin 34
#define InputB_Bin 35
#define ChannelA_Bin 18
#define ChannelB_Bin 36

// Solenoid (Use OUT3 and OUT4 on  MotorDriver "B")
#define Enable_Solenoid 37
#define InputA_Solenoid 38 //backup
#define InputB_Solenoid 39 //backup

// Upper Limit Switch
#define UpperSwitch 3


// Lower Limit Switch
#define LowerSwitch 2

// I2C Data Lines
#define SDA 20
#define SCL 21


void blink(){
  stop(); // stops elevator when switch is active
}

void Stop(){
  digitalWrite(InputA_Elevator,LOW);
  digitalWrite(InputB_Elevator,LOW);
}


// need to place a data parsing function for the I2C line




int BinByte;  // New Command for Bin Change after Data Parsing
int PrevBin;  // PrevBin Command for Bin Change algorithm
int SolenoidByte; // Servo Position for Sample Liquid
int DrillByte; // New Drill Operation Command byte
int ElevatorByte; // New Elevator Operation Command byte

int UpLimitSwitch = 0;  // Upper Limit Switch for Elevator
int DownLimitSwitch = 1; // Lower Limit Switch for Elevator


/*
  The following preprocessor definitions are values that
  are used to reset the Sample Return Module into its home
  state when the unit is powered up or has a unexpected power
  loss.

  The drill will be turned off, the elevator will be stowed
  in the up position, the bin will be returned to the 1st
  position, and the servo will be in its closed state.

  Preprocessors are used due to memory constraints with the
  AT2560 processsor.

*/

#define home_bin 0
#define home_elevator 1
#define home_drill 0
#define home_solenoid 0



int forward;
int backward;




void SolenoidControl(int i)
{
  if (i == 0)
  {
  analogWrite(Enable_Solenoid,255);
  }
  else if (i == 1)
  {
   analogWrite(Enable_Solenoid,255);
  }

}

void DrillControl(int i,int j)
{
 if(j == 0 || i == 0)
 {
  return;
 }
 if (j == 1)
 {
   if (i == 1)
   {
     digitalWrite(InputA_Drill,LOW);
     digitalWrite(InputB_Drill,HIGH);
   }
   else
   {
     digitalWrite(InputA_Drill, LOW);
     digitalWrite(InputB_Drill, LOW);
   }
 }

}
/*
 The operation of the bin requires an algorithm to determine the least amount of
 bins needed to pass in order to get from Bin A to Bin B.

 The function also preforms a check on the current status of the liquid
 container servo to determine if is closed or not. If the valve is in its open
 state, SolenoidControl() will be called to close the valve and recursivley call
 BinControl() again. If the servo is in its closed state, the servo will find
 the mimimum amount of bin changes required to achive the new bin.



 */
void BinControl(int i,int j)
{
  if (j == 1)
  {
   SolenoidControl(home_solenoid);
   delay(2000);
   j = 0;
   BinControl(i,j);
  }
  if (j == 0)
  {
   digitalWrite(InputA_Bin, HIGH);
   digitalWrite(InputB_Bin, LOW);
  }
}
void ElevatorControl(int i)
{
  if (i == 1)
  {
  while(UpLimitSwitch != 1)
  {
   digitalWrite(InputA_Elevator, HIGH);
   digitalWrite(InputB_Elevator,LOW);
  }
}
else if (i = 0)
{
  while(DownLimitSwitch != 1)
  {
    digitalWrite(InputA_Elevator, LOW);
    digitalWrite(InputB_Elevator,HIGH);
  }
}
}



void setup()
{
  Serial.begin(9600);
  pinMode(InputA_Bin, OUTPUT);
  pinMode(InputB_Bin, OUTPUT);
  pinMode(InputA_Drill, OUTPUT);
  pinMode(InputB_Drill, OUTPUT);
  pinMode(InputA_Elevator, OUTPUT);
  pinMode(InputB_Elevator, OUTPUT);
  pinMode(Enable_Solenoid, OUTPUT);
  pinMode(UpperSwitch, INPUT);
  pinMode(LowerSwitch, INPUT);
  attachInterrupt(digitalPinToInterrupt(UpperSwitch),blink,CHANGE); // hardware interupt for upper switch
  attachInterrupt(digitalPinToInterrupt(LowerSwitch),blink,CHANGE); // hardware interupt for lower switch
  while(UpLimitSwitch != 1)
  {
    SolenoidControl(home_solenoid);
    DrillControl(home_drill,home_elevator);
    BinControl(home_bin,home_solenoid);
    ElevatorControl(home_elevator);
  }
}

void loop()
{
 //ParseData();
  delay(5000);
}
