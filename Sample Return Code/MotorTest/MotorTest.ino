

#include <AFMotor.h>
#include <Wire.h>

AF_DCMotor motor_drill(1);
AF_DCMotor motor_elevator(4);
AF_DCMotor motor_bin(3);
AF_DCMotor motor(2);
static byte IncomingData[64];
static byte OutgoingData[64];


int BinByte;              // New Command for Bin Change after Data Parsing
int PrevBin;              // PrevBin Command for Bin Change algorithm
int SolenoidByte;         // Servo Position for Sample Liquid
int DrillByte;            // New Drill Operation Command byte
int ElevatorByte;         // New Elevator Operation Command byte

int UpLimitSwitch = 0;    // Upper Limit Switch for Elevator
int DownLimitSwitch = 1;  // Lower Limit Switch for Elevator


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
int z;
#define SDA A4
#define SCL A5

#define UpperSwitch 1
#define LowerSwitch 2
#define ChannelA_Bin 4
#define ChannelB_Bin 7

int p = 1;
int o = 0;


void Stop() {

  Serial.println("active");
}

byte DataParse(byte IncomingData[3], byte OutgoingData[3])
{
 if (IncomingData != NULL)
 {
 ElevatorByte = IncomingData[0];
 DrillByte = IncomingData[1];
 PrevBin = binByte;
 binByte = IncomingData[2];
 SolenoidByte = IncomingData[3];
 }
 
 ElevatorControl(ElevatorByte);
 DrillControl(DrillByte,ElevatorByte);
 BinControl(BinControl,SolenoidByte);
 SolenoidControl(SolenoidByte);


 OutgoingData[0] = ElevatorByte;
 OutgoingData[1] = DrillByte;
 OutgoingData[2] = binByte;
 OutgoingData[3] = SolenoidByte;

 return OutgoingData;
}




/*
    The solenoidControl() function below regulates the Solenoid speed,
    which takes 12V to be powered on (first part of the if statement).
    In addition, the solenoid is connected to 2 limit switches:
    when the limit switches are pressed the motor is turned off
    (second part of the if statement).
*/
void SolenoidControl(int i)
{
  if (i == 0)
  {
    motor.run(BACKWARD);
    for (i = 0; i < 255; i++) {
      motor.setSpeed(i);
      Serial.println("running solenoid");
    }
  }
  else if (i == 1)
  {
      motor.run(RELEASE);
      Serial.println("stop");
  }

}

void DrillControl(int i, int j) //to do
{
  if (j == 0 && i == 0)
  {
    return;
  }

  if (j == 1 && i == 0)
  {
    motor_drill.run(FORWARD);
    for (int i = 0; i < 255; i++)
      motor_drill.setSpeed(i);
    Serial.println("drill moving");
  }

  else if (j == 1 && i == 1)
  {
    motor_drill.run(FORWARD);
    for (int i = 0; i < 255; i++)
      motor_drill.setSpeed(i);
    Serial.println("drill moving");
  }

  else
  {
    motor_elevator.run(BACKWARD);
    for (int i = 0; i < 255; i++)
      motor_elevator.setSpeed(i);
    Serial.println("drill moving ");
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
void BinControl(int i, int j) //to do
{
  if (j == 0)
  {
    motor_bin.run(FORWARD);
    if (i == 0)
    {
     motor_bin.setSpeed(255);
    }
    else if (i == 1)
    {
     motor_bin.setSpeed(250);
    }
    else if (i == 2)
    {
     motor_bin.setSpeed(245);
    }
    else if (i == 3)
    {
     motor_bin.setSpeed(240);
    }
    else if (i == 4)
    {
     motor_bin.setSpeed(235);
    }
    else if (i == 5)
    {
     motor_bin.setSpeed(230);
    }
    else if (i == 6)
    {
     motor_bin.setSpeed(225);
    }
    else if (i == 7)
    {
     motor_bin.setSpeed(220);
    }
  }
  else if (j == 1)
  {
    SolenoidControl(home_solenoid);
    delay(2000);
    j = 0;
    BinControl(i, j);
  }
}

void ElevatorControl(int i)
{
  if (i == 0)
  {
    motor_elevator.run(BACKWARD);
    for (i = 0; i < 255; i++)
    {
      motor_elevator.setSpeed(i);
      Serial.println("running elevtor");
    }
  }
   else if (i == 2)
  {
      motor_elevator.run(RELEASE);
      Serial.println("stop");
  }
  else if (i == 1)
  {
    motor_elevator.run(FORWARD);
    for (i = 0; i < 255; i++)
    {
      motor_elevator.setSpeed(i);
      Serial.println("running up elevator");
    }
  }
}

void setup()
{
   Serial.begin(9600);           // set up Serial library at 9600 bps
   //pinMode(ChannelA_Bin, INPUT);
   //pinMode(ChannelB_Bin, INPUT);
   attachInterrupt(digitalPinToInterrupt(UpperSwitch), Stop, CHANGE);
   attachInterrupt(digitalPinToInterrupt(LowerSwitch), Stop, LOW);
   Serial.println("Motor test!");
   motor.setSpeed(200);

   //SolenoidControl(home_solenoid);
   //DrillControl(home_drill,home_elevator);
   //BinControl(home_bin,home_solenoid);
   //ElevatorControl(home_elevator);

   motor.run(RELEASE);
   Serial.println("Sample Return Module 2020 is active.");
}

void loop()
{
  //DataParse();
   SolenoidControl(o);
   delay(1000);
   ElevatorControl(p);
   delay(7000);
   ElevatorControl(o);
   SolenoidControl(p);
   delay(3000);
}
