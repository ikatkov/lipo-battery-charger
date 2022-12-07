#include <Arduino.h>
#include "U8g2lib.h"
#include "Countimer.h"
#include "NewTone.h"
#include "MCP4131.h"
#include <EncButton2.h>

#define HALF_STEP 0

const char NUMBER_FORMAT[] PROGMEM = "%6d";
const char FLOAT_FORMAT[] PROGMEM = "%.2f";
#define PRINT_DEC_POINTS 3
#define MIN_DIGIPOT_RESISTANCE 114
#define MAX_DIGIPOT_RESISTANCE 9870
#define EXTERNAL_RESISTOR 1000
#define MCU_VOLTAGE 5
#define SHUNT_RESISTANCE 0.15

static const uint32_t COMPUTE_POWER_INTERVAL = 1000; // ms

static const uint8_t MAX_CHARGE_TIME = 10;

volatile uint16_t current;
float voltage;
float power;

#define TONE_PIN 10
#define ROTARY_PIN_1 7
#define ROTARY_PIN_2 8
#define ROTARY_PIN_BTN 9

#define CH1_EN 17 // A3 since pin1&pin0 are HW UART i.e. Serial()
#define CH1_ISENSE 3

#define CH2_EN 2
#define CH3_EN 3

#define MCP4131_CH1_CS 4

EncButton2<EB_ENCBTN> enc(INPUT, 7, 8, 9);

//=================================
// UI
//=================================
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

MCP4131 potentiometer(MCP4131_CH1_CS);
Countimer timer;
uint16_t encoderPosition = 256;

void reDrawRegularScreen();

uint16_t computeMaxChargeCurrent()
{
    // I = 1200*1V/R
    float resistance = EXTERNAL_RESISTOR + MIN_DIGIPOT_RESISTANCE + (128 - encoderPosition) * (float(MAX_DIGIPOT_RESISTANCE) - MIN_DIGIPOT_RESISTANCE) / 128;
    return 1200 / resistance * 1000;
}

void readCurrent()
{
    uint16_t sensorValue = analogRead(A0);
    float shuntVoltage = (sensorValue + 0.5) * MCU_VOLTAGE / 1024;
    current = shuntVoltage / 20 / SHUNT_RESISTANCE * 1000;
    Serial.print("A0:");
    Serial.print(sensorValue);
    Serial.print("\tshuntVoltage:");
    Serial.print(shuntVoltage);
    Serial.print("\tcurrent:");
    Serial.println(current);
}

void reDrawRegularScreen()
{
    Serial.println(F("reDrawRegularScreen"));

    readCurrent();

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
}

void setup()
{
    pinMode(CH1_EN, OUTPUT);
    pinMode(CH2_EN, OUTPUT);
    pinMode(CH3_EN, OUTPUT);

    pinMode(CH1_EN, OUTPUT);
    pinMode(CH2_EN, OUTPUT);
    pinMode(CH3_EN, OUTPUT);

    digitalWrite(CH1_EN, LOW);
    digitalWrite(CH2_EN, LOW);
    digitalWrite(CH3_EN, LOW);

    Serial.begin(9600);

    while (!Serial)
    {
        delay(1);
    }

    Serial.println(F("booting up"));
    display.begin();

    potentiometer.writeWiper(128);

    timer.setCounter(MAX_CHARGE_TIME, 00, 0, CountType::COUNT_UP, NULL);
    timer.setInterval(reDrawRegularScreen, 500);
    timer.start();

    // ready beep
    NewTone(TONE_PIN, 750, 50);
    delay(100);
    NewTone(TONE_PIN, 1000, 50);
    delay(100);
    noNewTone(TONE_PIN);
}

// the loop function runs over and over again forever
void loop()
{
    timer.run();
    enc.tick();
    if (enc.turn())
    {
        Serial.println("turn");

        // можно опросить ещё:
        // Serial.println(enc.counter);  // вывести счётчик
        // Serial.println(enc.fast());   // проверить быстрый поворот
        Serial.println(enc.dir()); // направление поворота
        Serial.println(enc.counter);
        reDrawRegularScreen();

        // if (newPos > 128)
        // {
        //     newPos = 128;
        // }
        // else if (newPos < 0)
        // {
        //     newPos = 0;
        // }
        // // encoder->setPosition(newPos);
        // potentiometer.writeWiper(newPos);
        // encoderPosition = newPos;
        // Serial.print("pos:");
        // Serial.print(newPos);
        // Serial.print(" wiper:");
        // Serial.println(newPos);
    }
}
