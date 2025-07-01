#include <Arduino.h>

#define LED_RED 12 // GPIO12 = CMD_LED_RED

void setup()
{
  pinMode(LED_RED, OUTPUT);
}

void loop()
{
  digitalWrite(LED_RED, HIGH); // Allume la LED
  delay(500);                  // Attendre 500 ms
  digitalWrite(LED_RED, LOW);  // Éteint la LED
  delay(500);                  // Attendre 500 ms
}