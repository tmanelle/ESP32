#include <Arduino.h>
#include <SPI.h>
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
#define PIN_MOSI 23 // GPIO23 (SIO TMP126)
#define PIN_SCK 18  // GPIO18

// Répétitions maximales avant veille profonde
int repetitions = 0;
const int maxRepetitions = 5;

// Durée de veille (µs) en mode économie d'énergie
const uint64_t SLEEP_DURATION_US = 5ULL * 60ULL * 1000000ULL; // 5 minutes

// BLE UUIDs (Health Thermometer profile)
const char *serviceUUID = "00001809-0000-1000-8000-00805f9b34fb";
const char *charUUID = "00002A1C-0000-1000-8000-00805f9b34fb";

// Pointeur vers la caractéristique température
BLECharacteristic *pTempCharacteristic;

// Déclarations de fonctions
float readTemperatureTMP126();
void blinkIndicator(int pin, int toneValue);
void enterPowerSaveMode();

void setup()
{
  Serial.begin(115200);
  delay(100);

  // Initialisation des LEDs et buzzer
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_VERTE, OUTPUT);
  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER_PIN, 0);

  // Initialisation SPI pour TMP126
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // Initialisation BLE
  BLEDevice::init("ESP32_Temp_BLE");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(serviceUUID);

  pTempCharacteristic = pService->createCharacteristic(
      charUUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  pTempCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("BLE Thermometer prêt");
}

void loop()
{
  // Passage en veille si nombre de mesures atteint
  if (repetitions >= maxRepetitions)
  {
    enterPowerSaveMode();
  }

  // Lecture température
  float tempC = readTemperatureTMP126();
  Serial.printf("Température (°C) : %.2f\n", tempC);

  // Mise à jour BLE (format centi-degrés)
  int16_t tempInt = (int16_t)(tempC * 100);
  pTempCharacteristic->setValue((uint8_t *)&tempInt, sizeof(tempInt));
  pTempCharacteristic->notify();

  // Indication locale avec LEDs et buzzer
  blinkIndicator(LED_RED, 128);
  blinkIndicator(LED_VERTE, 128);

  repetitions++;

  // Entrée en veille entre cycles
  enterPowerSaveMode();
}

// Fonction de clignotement LED + buzzer
void blinkIndicator(int pin, int toneValue)
{
  digitalWrite(pin, HIGH);
  ledcWrite(0, toneValue);
  delay(500);
  digitalWrite(pin, LOW);
  ledcWrite(0, 0);
  delay(500);
}

// Lecture SPI TMP126
float readTemperatureTMP126()
{
  uint16_t rawValue;
  uint8_t msb, lsb;

  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x00);
  msb = SPI.transfer(0x00);
  lsb = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  rawValue = (msb << 8) | lsb;
  if (rawValue & 0x2000)
    rawValue |= 0xC000; // extension du signe
  int16_t signedTemp = (int16_t)rawValue;
  return signedTemp * 0.03125;
}

// Passage en veille profonde
void enterPowerSaveMode()
{
  Serial.println("Entering power save mode...");
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_VERTE, LOW);
  ledcWrite(0, 0);

  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  Serial.printf("Sleeping for %llu seconds...\n", SLEEP_DURATION_US / 1000000ULL);
  esp_deep_sleep_start();
}