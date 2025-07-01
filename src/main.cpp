#include <Arduino.h>

// Choisir la bonne broche pour la LED
#define LED_PIN 2 // essaie GPIO2, souvent utilisée par défaut

void setup()
{
  pinMode(LED_PIN, OUTPUT); // Configure la pin en sortie
}

void loop()
{
  digitalWrite(LED_PIN, HIGH); // Allume la LED
  delay(500);                  // Pause de 500 ms
  digitalWrite(LED_PIN, LOW);  // Éteint la LED
  delay(500);                  // Pause de 500 ms
}
