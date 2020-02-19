#include <Wire.h> 



//Elevator Motor Pins
int enA = 30; 
int in1 = 9; //DOWN
int in2 = 8; //UP

// Auger Motor Pins
int enB = 31;
int in3 = 7; //
int in4 = 6;

//Solenoid Pin
int in5 = 13;

//Bin Motor 
int in6 = 10;
int in7 = 11;
int in8 = 12;

//I2C Lines
int inData = 20;
int inClock = 21;
int elevspeed;
int drillspeed;
int rev_speed; 


void setup()
{
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  void HomeState(); 
  drillspeed = 0; 
  elevspeed = -255; 

  //Serial.println("Arduino is ready");
  //Serial.println("Enter a number between 0 and 255. Negative numbers reverse direction.");
}

void loop()
{
  //int array[10] = {-200,0,200,0,-200,0,200,0,200,0};
    //motorspeed = array[i];
  if (elevspeed >= 0 && elevspeed <= 255) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, elevspeed);
    delay(3000);
    analogWrite(enB, elevspeed);
    //Serial.print("Motor Speed: ");
    //Serial.println(motorspeed);
  }
  else if (elevspeed < 0 && elevspeed >= -255) {
    rev_speed = elevspeed;
    drillspeed = abs(elevspeed); 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, rev_speed);
    delay(5000);
    analogWrite(enB, rev_speed);
  }

  if (drillspeed > 0 && drillspeed <= 255) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, elevspeed);
    delay(3000);
    analogWrite(enB, elevspeed);
    //Serial.print("Motor Speed: ");
    //Serial.println(motorspeed);
  }

  if (drillspeed == 0)
  {
    digitalWrite(in1,LOW); 
    digitalWrite(in2,HIGH);
    analogWrite(enA, 255);
    delay(3000); 
  }
}
