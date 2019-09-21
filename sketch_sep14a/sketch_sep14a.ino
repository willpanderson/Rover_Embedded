int potPin = A0; //The potentiometer pin is analog A0
int LedPin = 0;  //the led pin in arduino is pin 9
int readValue;  //variable used for writing to the led
int writeValue; //variable for writing led

void setput()
{
  pinMode(potPin, INPUT);  //sets the potPin to be an input
  pinMode(LedPin, OUTPUT); //sets the led pin to be an output
  Serial.begin(9600);      // turns on the Serial Port
}

void loop() 
{
  readValue = analogRead(potPin); //Reads the V on the potentiometer
  writeValue = (255./1023.) * readValue; // calculate the value for the led
  analogWrite(LedPin, writeValue);  //write to the LED
  Serial.print("You are writing a value of "); //for debugging print your values 
  Serial.println(writeValue);
}
