/* #include <Arduino.h>
#include <SPI.h>

// Broches de commande
#define LED_RED     12   // GPIO12
#define LED_VERTE   17   // GPIO19
#define BUZZER_PIN  13   // GPIO13

// Broches SPI TMP126 (SIO = MOSI unique)
#define PIN_CS      5    // GPIO5
#define PIN_MOSI    23   // GPIO23 (SIO TMP126)
#define PIN_SCK     18   // GPIO18

// Déclaration de la fonction
float readTemperatureTMP126();

// Répétition contrôlée
int repetitions = 0;
const int maxRepetitions = 5;

void setup()
{
  Serial.begin(115200);

  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_VERTE, OUTPUT);

  // Buzzer (canal 0, 2 kHz, 8 bits)
  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER_PIN, 0);

  // SPI (SIO sur MOSI, pas de MISO)
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH); // CS inactif
}

void loop()
{
  if (repetitions >= maxRepetitions) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_VERTE, LOW);
    ledcWrite(0, 0);
    while (true) {} // arrêt total
  }

  // Lire température
  float tempC = readTemperatureTMP126();
  Serial.print("Température (°C) : ");
  Serial.println(tempC);

  // Clignotement LED rouge + buzzer
  digitalWrite(LED_RED, HIGH);
  ledcWrite(0, 128);
  delay(500);
  digitalWrite(LED_RED, LOW);
  ledcWrite(0, 0);
  delay(500);

  // Clignotement LED verte + buzzer
  digitalWrite(LED_VERTE, HIGH);
  ledcWrite(0, 128);
  delay(500);
  digitalWrite(LED_VERTE, LOW);
  ledcWrite(0, 0);
  delay(500);

  repetitions++;
}

// Lecture SPI TMP126 (registre 0x00, SIO = MOSI)
float readTemperatureTMP126()
{
  uint16_t rawValue = 0;
  uint8_t msb = 0, lsb = 0;

  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x00);             // Adresse registre température
  msb = SPI.transfer(0x00);       // MSB
  lsb = SPI.transfer(0x00);       // LSB
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  rawValue = (msb << 8) | lsb;

  Serial.print("Raw SPI value: 0x");
  Serial.println(rawValue, HEX);

  // Extension signe pour 14 bits si nécessaire
  if (rawValue & 0x2000) {
    rawValue |= 0xC000;
  }

  int16_t signedTemp = (int16_t)rawValue;
  return signedTemp * 0.03125;
}
*/

#include <Arduino.h>
#include <SPI.h>

#define PIN_CS 5
#define PIN_SCK 18
#define PIN_MOSI 23

void setup()
{
  Serial.begin(115200);
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  delay(500);
  Serial.println("Test TMP126 - lecture 2e cycle");
}

void loop()
{
  uint8_t response1_MSB = 0, response1_LSB = 0;
  uint8_t response2_MSB = 0, response2_LSB = 0;
  uint16_t rawValue = 0;

  // 1er cycle — envoie commande, récupère réponse inutile
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x00);                 // adresse registre
  response1_MSB = SPI.transfer(0x00); // ignore
  response1_LSB = SPI.transfer(0x00); // ignore
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  delayMicroseconds(10); // très court délai

  // 2e cycle — cette fois, on récupère la température
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x00); // re-lire même registre
  response2_MSB = SPI.transfer(0x00);
  response2_LSB = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  rawValue = (response2_MSB << 8) | response2_LSB;

  Serial.print("R1 (ignored): 0x");
  Serial.print(response1_MSB, HEX);
  Serial.print(" ");
  Serial.print(response1_LSB, HEX);
  Serial.print(" | R2: 0x");
  Serial.print(response2_MSB, HEX);
  Serial.print(" ");
  Serial.print(response2_LSB, HEX);
  Serial.print(" | RAW: 0x");
  Serial.print(rawValue, HEX);

  // Convertir
  if (rawValue & 0x2000)
  {
    rawValue |= 0xC000;
  }
  int16_t signedTemp = (int16_t)rawValue;
  float tempC = signedTemp * 0.03125;

  Serial.print(" | Température : ");
  Serial.print(tempC);
  Serial.println(" °C");

  delay(1000);
}