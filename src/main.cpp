#include <Arduino.h>

#define LED_RED 12   // GPIO12 = CMD_LED_RED
#define LED_VERTE 19 // GPIO19 = CMD_LED_VERTE

void setup()
{
  pinMode(LED_RED, OUTPUT);
}

void loop()
{
  // Clignote LED rouge
  digitalWrite(LED_RED, HIGH); // Allume la LED
  delay(500);                  // Attendre 500 ms
  digitalWrite(LED_RED, LOW);  // Éteint la LED
  delay(500);                  // Attendre 500 ms

  // Clignote LED verte
  digitalWrite(LED_VERTE, HIGH);
  delay(500);
  digitalWrite(LED_VERTE, LOW);
  delay(500);
}