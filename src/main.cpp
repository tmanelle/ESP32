/* #include <Arduino.h>

#define LED_RED 12
#define LED_VERTE 19
#define BUZZER 21

void setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_VERTE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
}

void loop()
{
  // Clignotement LED rouge
  digitalWrite(LED_RED, HIGH);
  delay(500);
  digitalWrite(LED_RED, LOW);
  delay(500);

  // Clignotement LED verte
  digitalWrite(LED_VERTE, HIGH);
  delay(500);
  digitalWrite(LED_VERTE, LOW);
  delay(500);

  // Test buzzer manuel
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(BUZZER, HIGH);
    delay(1);
    digitalWrite(BUZZER, LOW);
    delay(1);
  }

  delay(1000);
}
*/

#include <Arduino.h>

// Définition des broches
#define LED_RED 12    // GPIO12 = CMD_LED_RED
#define LED_VERTE 19  // GPIO19 = CMD_LED_VERTE
#define BUZZER_PIN 13 // GPIO13 = Buzzer-PWM

void setup()
{
  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_VERTE, OUTPUT);

  // Configuration du buzzer en PWM (canal 0, fréquence 2 kHz, 8 bits)
  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER_PIN, 0);
}

void loop()
{
  // Allume LED rouge et buzzer
  digitalWrite(LED_RED, HIGH);
  ledcWrite(0, 128); // Active le buzzer à 50% de volume
  delay(500);

  // Éteint LED rouge et buzzer
  digitalWrite(LED_RED, LOW);
  ledcWrite(0, 0); // Stoppe le buzzer
  delay(500);

  // Allume LED verte et buzzer
  digitalWrite(LED_VERTE, HIGH);
  ledcWrite(0, 128);
  delay(500);

  // Éteint LED verte et buzzer
  digitalWrite(LED_VERTE, LOW);
  ledcWrite(0, 0);
  delay(500);
}
