int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
float photocellReading;    // the analog reading from the sensor divider
float convert;
int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  photocellReading = analogRead(photocellPin);
  
  convert = (photocellReading / 1023 );
  LEDbrightness = convert * 255;
  
 // photocellReading = 1023 - photocellReading;
  //LEDbrightness = map(photocellReading, 0, 1023, 0, 255);  
 
  if (Serial.available() > 0)
  {
    Serial.println(convert);
    Serial.println(photocellReading);
    Serial.println(LEDbrightness);
    Serial.read();
  }
}
