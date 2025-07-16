#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_sleep.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Broches de commande
#define LED_RED 12    // GPIO12
#define LED_VERTE 17  // GPIO17
#define BUZZER_PIN 13 // GPIO13

// Broches SPI TMP126 (SIO = MOSI unique)
#define PIN_CS 5    // GPIO5
#define PIN_MOSI 23 // GPIO23
#define PIN_SCK 18  // GPIO18

// Boutons poussoirs
#define BUTTON1_PIN 0  // GPIO0 (UPLOAD)
#define BUTTON2_PIN 15 // GPIO15 (mode light sleep)
bool button1Pressed = false;
bool button2Pressed = false;

// BLE UUIDs
const char *serviceUUID = "00001809-0000-1000-8000-00805f9b34fb";
const char *charUUID_TEMPERATURE = "00002A1C-0000-1000-8000-00805f9b34fb";
const char *charUUID_CURRENT = "00002A19-0000-1000-8000-00805f9b34fb";

// Pointeurs vers les caractéristiques BLE
BLECharacteristic *pTempCharacteristic;
BLECharacteristic *pCurrentCharacteristic;

// INA237 I2C address
#define INA237_ADDR 0x40

//------------------------------------------------------------------------------
// I2C helper for INA237
void writeRegister(uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(INA237_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(INA237_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA237_ADDR, (uint8_t)2);
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (hi << 8) | lo;
}

// Initialise INA237
void initINA237()
{
  writeRegister(0x00, 0x2027); // CONFIG
  writeRegister(0x05, 0x0190); // CALIBRATION
}

// Lit le courant en A
float readCurrent_A()
{
  int16_t raw = (int16_t)readRegister(0x04);
  return raw * 100e-6; // 100µA LSB
}

// Prototypes
float readTemperatureTMP126();
void blinkIndicator(int pin, int toneValue);
void enterLightSleep();

void setup()
{
  Serial.begin(115200);
  delay(100);

  // LEDs et buzzer
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_VERTE, OUTPUT);
  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER_PIN, 0);

  // Boutons
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // SPI TMP126
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // I2C + INA237
  Wire.begin(21, 22);
  initINA237();

  // BLE
  BLEDevice::init("ESP32_Temp_Current");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(serviceUUID);

  pTempCharacteristic = service->createCharacteristic(
      charUUID_TEMPERATURE,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pTempCharacteristic->addDescriptor(new BLE2902());

  pCurrentCharacteristic = service->createCharacteristic(
      charUUID_CURRENT,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCurrentCharacteristic->addDescriptor(new BLE2902());

  service->start();
  server->getAdvertising()->start();
  Serial.println("BLE Thermometer + Current ready");
}

void loop()
{
  // Bouton Upload
  if (digitalRead(BUTTON1_PIN) == LOW)
  {
    Serial.println("Button 1 (UPLOAD) pressed");
    button1Pressed = true;
    blinkIndicator(LED_RED, 200);
  }

  // Bouton Light Sleep
  if (digitalRead(BUTTON2_PIN) == LOW)
  {
    Serial.println("Button 2 (Light Sleep) pressed");
    button2Pressed = true;
    blinkIndicator(LED_VERTE, 200);
    enterLightSleep();
  }

  // Lecture température
  float tempC = readTemperatureTMP126();
  Serial.printf("Temperature: %.2f C\n", tempC);
  int16_t tempInt = (int16_t)(tempC * 100);
  pTempCharacteristic->setValue((uint8_t *)&tempInt, sizeof(tempInt));
  pTempCharacteristic->notify();

  // Lecture courant
  float currentA = readCurrent_A();
  Serial.printf("Current: %.3f A\n", currentA);
  int16_t currInt = (int16_t)(currentA * 1000);
  pCurrentCharacteristic->setValue((uint8_t *)&currInt, sizeof(currInt));
  pCurrentCharacteristic->notify();

  // Indicateurs
  blinkIndicator(LED_RED, 128);
  blinkIndicator(LED_VERTE, 128);

  // Pas de mode économie automatique
  delay(1000); // temporisation entre lectures
}

void blinkIndicator(int pin, int toneValue)
{
  digitalWrite(pin, HIGH);
  ledcWrite(0, toneValue);
  delay(300);
  digitalWrite(pin, LOW);
  ledcWrite(0, 0);
  delay(200);
}

void enterLightSleep()
{
  Serial.println("Entering light sleep...");
  esp_sleep_enable_timer_wakeup(5ULL * 60ULL * 1000000ULL);
  esp_light_sleep_start();
  Serial.println("Woke up from light sleep");
}

float readTemperatureTMP126()
{
  uint16_t raw;
  uint8_t msb, lsb;
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x00);
  msb = SPI.transfer(0x00);
  lsb = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  raw = (msb << 8) | lsb;
  if (raw & 0x2000)
    raw |= 0xC000;
  int16_t st = (int16_t)raw;
  return st * 0.03125;
}
