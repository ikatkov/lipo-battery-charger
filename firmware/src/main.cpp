#include <Arduino.h>
#include "U8g2lib.h"
#include "Countimer.h"
#include "NewTone.h"
#include "MCP4551.h"
#include "RotaryEncoder.h"

#define HALF_STEP 0

const char NUMBER_FORMAT[] PROGMEM = "%6d";
const char FLOAT_FORMAT[] PROGMEM = "%.2f";
#define PRINT_DEC_POINTS 3
#define MIN_DIGIPOT_RESISTANCE 113
#define MAX_DIGIPOT_RESISTANCE 9670
#define EXTERNAL_RESISTOR 1000
#define MCU_VOLTAGE 3.3
#define SHUNT_RESISTANCE 0.15

static const uint32_t COMPUTE_POWER_INTERVAL = 1000; // ms

static const uint8_t MAX_CHARGE_TIME = 10;

volatile uint16_t current;
float voltage;
float power;

static const byte TONE_PIN = 4;
static const byte ROTARY_PIN_1 = 2;
static const byte ROTARY_PIN_2 = 3;

//=================================
// UI
//=================================
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
RotaryEncoder *encoder = nullptr;

MCP4551 myMCP4551;
Countimer timer;
uint16_t encoderPosition = 256;

void reDrawRegularScreen();

uint16_t computeMaxChargeCurrent()
{
  // I = 1200*1V/R
  float resistance = EXTERNAL_RESISTOR + MIN_DIGIPOT_RESISTANCE + (256 - encoderPosition) * (float(MAX_DIGIPOT_RESISTANCE) - MIN_DIGIPOT_RESISTANCE) / 256;
  return 1200 / resistance * 1000;
}

void reDrawRegularScreen()
{
  // Serial.println(F("reDrawRegularScreen"));

  char buffer[10];
  display.firstPage();
  do
  {
    // current
    display.setFont(u8g2_font_helvB14_tr);
    snprintf_P(buffer, 7, NUMBER_FORMAT, current);
    display.drawStr(60, 16, buffer);
    display.setFont(u8g2_font_6x13_tr);
    display.drawStr(115, 16, "mA");

    // limit current
    display.setFont(u8g2_font_helvB14_tr);
    // Serial.println(current[selectedChannel]);
    snprintf_P(buffer, 7, NUMBER_FORMAT, computeMaxChargeCurrent());
    display.drawStr(60, 32, buffer);
    display.setFont(u8g2_font_6x13_tr);
    display.drawStr(115, 32, "mA");

    //==============================
    // mAh
    //==============================
    display.setDrawColor(1);
    display.setFont(u8g2_font_helvB14_tr);
    snprintf_P(buffer, 7, NUMBER_FORMAT, (int)round(power));
    display.drawStr(0, 18, buffer);
    display.setFont(u8g2_font_6x13_tr);
    display.drawStr(20, 32, "mAh");

  } while (display.nextPage());
}

// void computePower()
// {
//   Serial.println(F("computePower"));
//   for (int channel = 0; channel < 3; channel++)
//   {
//     if (state == CHARGING_STATE)
//     {
//       power = power + (1.0f * current * 10 * COMPUTE_POWER_INTERVAL / 1000 / 60 / 60);
//       Serial.print(F("Channel "));
//       Serial.print(channel);
//       Serial.print(" power is ");
//       Serial.println(power);
//     }
//   }
// }

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

void setup()
{
  Serial.begin(38400);

  while (!Serial)
  {
    delay(1);
  }

  Serial.println(F("booting up"));
  display.begin();

  myMCP4551.begin();
  myMCP4551.setWiper(256);

  timer.setCounter(MAX_CHARGE_TIME, 00, 0, CountType::COUNT_UP, NULL);
  // timer.setInterval(computePower, 1000);
  timer.start();

  encoder = new RotaryEncoder(ROTARY_PIN_1, ROTARY_PIN_2, RotaryEncoder::LatchMode::TWO03);
  encoder->setPosition(256);
  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_2), checkPosition, CHANGE);

  // ready beep
  NewTone(TONE_PIN, 750, 50);
  delay(100);
  NewTone(TONE_PIN, 1000, 50);
}

// the loop function runs over and over again forever
void loop()
{
  timer.run();
  reDrawRegularScreen();
  uint16_t sensorValue = analogRead(A0);
  float shuntVoltage = (sensorValue + 0.5) * MCU_VOLTAGE / 1024;
  current = shuntVoltage/ 20 / SHUNT_RESISTANCE *1000;
  Serial.print("A0:");
  Serial.print(sensorValue);
  Serial.print("\tshuntVoltage:");
  Serial.print(shuntVoltage);
  Serial.print("\tcurrent:");
  Serial.println(current);

  // encoder->tick(); // just call tick() to check the state.
  long newPos = encoder->getPosition();
  if (encoderPosition != newPos)
  {
    if (newPos > 256)
    {
      newPos = 256;
    }
    else if (newPos < 0)
    {
      newPos = 0;
    }
    encoder->setPosition(newPos);
    myMCP4551.setWiper(newPos);
    encoderPosition = newPos;
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" wiper:");
    Serial.println(newPos);
  } // if

   delay(50);
}
