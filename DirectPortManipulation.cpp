
#define PIN 2

#include <Arduino.h>
void loop() {
  GPIO.out_w1ts= (1 << PIN); //Set HIGH
  delay(1000);
  GPIO.out_w1tc= (1 << PIN); //Set HIGH
  delay(1000);
}
