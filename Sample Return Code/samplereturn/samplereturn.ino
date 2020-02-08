int enA = 30;
int in1 = 9;
int in2 = 8;
// motor two
int enB = 10;
int in3 = 7;
int in4 = 6;
int motorspeed;


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
  if (Serial.available()) {
    motorspeed = Serial.parseInt();
    Serial.println(motorspeed);

  }
  if (motorspeed >= 0 && motorspeed <= 255) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, motorspeed);
    analogWrite(enB, motorspeed);
    //Serial.print("Motor Speed: ");
    //Serial.println(motorspeed);
  }
  if (motorspeed < 0 && motorspeed >= -255) {
    int rev_speed = abs(motorspeed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, rev_speed);
    analogWrite(enB, rev_speed);
  }
}
