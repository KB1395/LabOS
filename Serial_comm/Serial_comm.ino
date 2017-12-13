int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;    // the analog reading from the sensor divider
int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  photocellReading = analogRead(photocellPin);
  

  if (Serial.available() > 0)
  {
    Serial.println(photocellReading);
    Serial.read();
  }
}
