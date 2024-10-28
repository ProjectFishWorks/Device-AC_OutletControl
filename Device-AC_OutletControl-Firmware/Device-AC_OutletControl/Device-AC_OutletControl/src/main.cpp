#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;
int I2C_SDA = 3;
int I2C_SCL = 2;
// node controller core object
// NodeControllerCore core;
const float FACTOR = 15; // 20A/1V from teh CT

const float multiplier = 0.00005;

float getcurrent();

void setup()
{
  Serial.begin(115200);
    // initialize digital pin LED_BUILTIN as an output.
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  ads.setGain(GAIN_FOUR); // +/- 1.024V 1bit = 0.5mV
  ads.begin();
}

void printMeasure(String prefix, float value, String postfix)
{
  Serial.print(prefix);
  Serial.print(value, 3);
  Serial.println(postfix);
}

void loop()
{
   printMeasure("Irms: ", getcurrent(), "A");
  digitalWrite(0, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(5000); 
   printMeasure("Irms: ", getcurrent(), "A");                      // wait for a second
  digitalWrite(0, LOW);    // turn the LED off by making the voltage LOW
  delay(5000);   
   printMeasure("Irms: ", getcurrent(), "A");                    // wait for a second
  digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000); 
   printMeasure("Irms: ", getcurrent(), "A");                      // wait for a second
  digitalWrite(1, LOW);    // turn the LED off by making the voltage LOW
  delay(2000);
   printMeasure("Irms: ", getcurrent(), "A");

  printMeasure("Irms: ", getcurrent(), "A");
  delay(1000);
}

float getcurrent()
{
  float voltage;
  float current;
  float sum = 0;
  long time_check = millis();
  int counter = 0;

  while (millis() - time_check < 1000)
  {
    voltage = ads.readADC_Differential_0_1() * multiplier;
    current = voltage * FACTOR;
    // current /= 1000.0;

    sum += sq(current);
    counter = counter + 1;
  }

  current = sqrt(sum / counter);
  return (current);
}