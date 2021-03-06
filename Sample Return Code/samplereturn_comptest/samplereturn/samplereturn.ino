#include <Wire.h> 



//Elevator Motor Pins
int enA = 38;
int in1 = 2;
int in2 = 3;
// Auger Motor Pins
int enB = 31;
int in3 = 7;
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
int motorspeed;
int drillspeed;


void setup()
{
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  

  Serial.println("Arduino is ready");
  Serial.println("Enter a number between 0 and 255. Negative numbers reverse direction.");
}

void loop()
{
  //int array[10] = {-255,0,255,0,-255,0,255,0,255,0};
  //for (int i = 0; i < 11; i++)
  //{
    motorspeed = 255;
    drillspeed = 255;
  if (motorspeed >= 0 && motorspeed <= 255) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, motorspeed);
    delay(3000);
    analogWrite(enB, motorspeed);
    //Serial.print("Motor Speed: ");
    //Serial.println(motorspeed);
  }
  else if (motorspeed < 0 && motorspeed >= -255) {
    int rev_speed = abs(motorspeed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, rev_speed);
    delay(5000);
    analogWrite(enB, rev_speed);
  }
  if (drillspeed >= 0 && drillspeed <= 255) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, motorspeed);
    delay(3000);
    analogWrite(enB, motorspeed);
    //Serial.print("Motor Speed: ");
    //Serial.println(motorspeed);
  }
  else if (drillspeed < 0 && drillspeed >= -255) {
    int revq_speed = abs(drillspeed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, revq_speed);
    delay(3000);
    analogWrite(enB, revq_speed);
  }
}
//}
