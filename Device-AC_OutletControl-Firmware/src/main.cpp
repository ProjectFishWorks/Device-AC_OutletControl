#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include <NodeControllerCore.h>

#define NEOPIXEL_PIN 10
#define NUMPIXELS 4 // Update to 4 NeoPixels

#define RELAY_1_NC_PIN 0
#define RELAY_2_NO_PIN 1

#define NODE_ID 164                        // 164
#define RELAY_1_NC_MESSAGE_ID 0x0A00       // 2560
#define RELAY_2_NO_MESSAGE_ID 0x0A01       // 2561
#define CURRENT_MESSAGE_ID 0x0A02          // 2562
#define RELAY_1_ALERT_NODE_MESSAGE_ID 2563 // 2563
#define RELAY_2_ALERT_NODE_MESSAGE_ID 2564 // 2564

#define mesageDelay 1000

bool is_Relay_1_in_Alert = false;
bool is_Relay_2_in_Alert = false;

int relay_1_alert_node = -1;
int relay_2_alert_node = -1;

float total_current = 0;

Adafruit_ADS1115 ads;
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

int I2C_SDA = 3;
int I2C_SCL = 2;

const float FACTOR = 15; // 15A/1V from the CT
const float multiplier = 0.00005;

bool relay0State = true;
bool relay1State = true;

NodeControllerCore core;

//-------------------------------------------------- Function Prototypes --------------------------------------------------

void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

void getcurrent(void *parameters);

void printMeasure(String prefix, float value, String postfix);

//-------------------------------------------------- setup() --------------------------------------------------

void setup()
{
  Serial.begin(115200);
  // Set initial relay states to ON
  core = NodeControllerCore();
  if (core.Init(receive_message, NODE_ID))
  {
    Serial.println("Driver device initialized");
  }
  else
  {
    Serial.println("Failed to initialize driver");
  }

  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);

  // initialize digital pins 0 and 1 as outputs
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  // Initialize I2C and ADS1115
  Wire.begin(I2C_SDA, I2C_SCL);
  ads.setGain(GAIN_FOUR); // +/- 1.024V 1bit = 0.5mV
  ads.begin();

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(3); // Set brightness to 1% (90% less than current setting) // Set brightness to 10%
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Set first NeoPixel to green (relay 0 active)
  pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // Set second NeoPixel to green (relay 0 active)
  pixels.setPixelColor(2, pixels.Color(0, 255, 0)); // Set third NeoPixel to green (relay 1 active)
  pixels.setPixelColor(3, pixels.Color(0, 255, 0)); // Set fourth NeoPixel to green (relay 1 active)
  pixels.show();

  // Create a task to get the current
  xTaskCreate(getcurrent, "getcurrent", 4096, NULL, 1, NULL);
}

//-------------------------------------------------- loop() --------------------------------------------------

void loop()
{
  // float current = getcurrent();
  // printMeasure("Irms: ", current, "A");

  if (Serial.available() > 0)
  {
    char key = Serial.read();
    if (key == '1')
    {
      relay0State = !relay0State; // Toggle relay 0 state
      digitalWrite(0, relay0State ? HIGH : LOW);
      if (relay0State)
      {
        pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Set first NeoPixel to green (relay 0 active)
        pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // Set second NeoPixel to green (relay 0 active)
      }
      else
      {
        pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Set first NeoPixel to red (relay 0 inactive)
        pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // Set second NeoPixel to red (relay 0 inactive)
      }
      pixels.show();
    }
    else if (key == '2')
    {
      relay1State = !relay1State; // Toggle relay 1 state
      digitalWrite(1, relay1State ? HIGH : LOW);
      if (relay1State)
      {
        pixels.setPixelColor(2, pixels.Color(0, 255, 0)); // Set third NeoPixel to green (relay 1 active)
        pixels.setPixelColor(3, pixels.Color(0, 255, 0)); // Set fourth NeoPixel to green (relay 1 active)
      }
      else
      {
        pixels.setPixelColor(2, pixels.Color(255, 0, 0)); // Set third NeoPixel to red (relay 1 inactive)
        pixels.setPixelColor(3, pixels.Color(255, 0, 0)); // Set fourth NeoPixel to red (relay 1 inactive)
      }
      pixels.show();
    }
  }
  delay(100);
}

//-------------------------------------------------- Function Definitions --------------------------------------------------
void getcurrent(void *parameters)
{
  Serial.println("Getting current");

  while (1)
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

      sum += sq(current);
      counter = counter + 1;
    }

    total_current = sqrt(sum / counter);
    core.sendMessage(CURRENT_MESSAGE_ID, &total_current);
    Serial.print("Current: ");
    Serial.println(total_current);
    delay(4000);
  }
}

void printMeasure(String prefix, float value, String postfix)
{
  Serial.print(prefix);
  Serial.print(value, 3);
  Serial.println(postfix);
}

void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data)
{
  Serial.println("Message received callback");

  // Check if the message is for this node
  if (nodeID == NODE_ID)

  {
    Serial.println("Message received to self");
    // Check the message ID for the LED control messages
    switch (messageID)
    {
      // ---------------------Demo override control messages-------------------------
    case RELAY_1_NC_MESSAGE_ID:
      if (data == 1)
      {
        if (!is_Relay_1_in_Alert)
        {
          digitalWrite(RELAY_1_NC_PIN, LOW);
          pixels.setPixelColor(0, pixels.Color(0, 255, 0));
          pixels.setPixelColor(1, pixels.Color(0, 255, 0));
          pixels.show();
          Serial.println("Relay 1 is ON");
          delay(mesageDelay);
        }
      }
      else
      {
        if (!is_Relay_1_in_Alert)
        {
          digitalWrite(RELAY_1_NC_PIN, HIGH);
          pixels.setPixelColor(0, pixels.Color(255, 0, 0));
          pixels.setPixelColor(1, pixels.Color(255, 0, 0));
          pixels.show();
          Serial.println("Relay 1 is OFF");
          delay(mesageDelay);
        }
        break;

      case RELAY_2_NO_MESSAGE_ID:
        if (data == 1)
        {
          if (!is_Relay_2_in_Alert)
          {
            digitalWrite(RELAY_2_NO_PIN, HIGH);
            pixels.setPixelColor(2, pixels.Color(0, 255, 0));
            pixels.setPixelColor(3, pixels.Color(0, 255, 0));
            pixels.show();
            Serial.println("Relay 2 is ON");
          }
          delay(mesageDelay);
        }
        else
        {
          if (!is_Relay_2_in_Alert)
          {
            digitalWrite(RELAY_2_NO_PIN, LOW);
            pixels.setPixelColor(2, pixels.Color(255, 0, 0));
            pixels.setPixelColor(3, pixels.Color(255, 0, 0));
            pixels.show();
            Serial.println("Relay 2 is OFF");
            delay(mesageDelay);
          }
        }
        break;
      case RELAY_1_ALERT_NODE_MESSAGE_ID:
        relay_1_alert_node = data;
        Serial.print("Relay 1 alert node: ");
        Serial.println(relay_1_alert_node);
        break;
      case RELAY_2_ALERT_NODE_MESSAGE_ID:
        relay_2_alert_node = data;
        Serial.print("Relay 2 alert node: ");
        Serial.println(relay_2_alert_node);
        break;

      default:
        break;
      }
    }
  }
  if (nodeID == relay_1_alert_node)
  {
    if (messageID == 901 || messageID == 900)
    {
      if (data == 1)
      {
        is_Relay_1_in_Alert = true;
        digitalWrite(RELAY_1_NC_PIN, LOW);
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.setPixelColor(1, pixels.Color(255, 0, 0));
        pixels.show();
        Serial.println("Relay 1 is in alert");
      }
      else
      {
        is_Relay_1_in_Alert = false;
        digitalWrite(RELAY_1_NC_PIN, HIGH);
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.setPixelColor(1, pixels.Color(0, 255, 0));
        pixels.show();
        Serial.println("Relay 1 is out of alert");
      }
    }
  }
  if (nodeID == relay_2_alert_node)
  {
    if (messageID == 901 || messageID == 900)
    {
      if (data == 1)
      {
        is_Relay_2_in_Alert = true;
        digitalWrite(RELAY_2_NO_PIN, LOW);
        pixels.setPixelColor(2, pixels.Color(255, 0, 0));
        pixels.setPixelColor(3, pixels.Color(255, 0, 0));
        pixels.show();
        Serial.println("Relay 2 is in alert");
      }
      else
      {
        is_Relay_2_in_Alert = false;
        digitalWrite(RELAY_2_NO_PIN, HIGH);
        pixels.setPixelColor(2, pixels.Color(0, 255, 0));
        pixels.setPixelColor(3, pixels.Color(0, 255, 0));
        pixels.show();
        Serial.println("Relay 2 is out of alert");
      }
    }
  }
}
