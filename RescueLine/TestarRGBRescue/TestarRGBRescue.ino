#include <Wire.h>
#include "Adafruit_TCS34725softi2c.h"

const int sdaDir = 48, sclDir = 46, sdaEsq = 44, sclEsq = 42;

Adafruit_TCS34725softi2c rgbDir = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, sdaDir, sclDir);
Adafruit_TCS34725softi2c rgbEsq = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, sdaEsq, sclEsq);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  while (!rgbDir.begin()) {
    Serial.println("O sensor RGB direito não foi encontrado ... verifica as ligações");
    delay(1000);
  }

  while (!rgbEsq.begin()) {
    Serial.println("O sensor RGB esquerdo não foi encontrado ... verifica as ligações");
    delay(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  checkColor();
}

void checkColor() {

  uint16_t clearDir, redDir, greenDir, blueDir;
  uint16_t clearEsq, redEsq, greenEsq, blueEsq;
  rgbDir.setInterrupt(false);  // turn on LED
  rgbEsq.setInterrupt(false);  // turn on LED
  delay(60);                   // takes 50ms to read
  rgbDir.getRawData(&redDir, &greenDir, &blueDir, &clearDir);
  rgbEsq.getRawData(&redEsq, &greenEsq, &blueEsq, &clearEsq);
  rgbDir.setInterrupt(true);  // turn off LED
  rgbEsq.setInterrupt(true);  // turn off LED

  uint32_t sumDir = clearDir;
  float dirR, dirG, dirB;
  dirR = redDir;
  dirR /= sumDir;
  dirG = greenDir;
  dirG /= sumDir;
  dirB = blueDir;
  dirB /= sumDir;
  dirR *= 256;
  dirG *= 256;
  dirB *= 256;

  uint32_t sumEsq = clearEsq;
  float esqR, esqG, esqB;
  esqR = redEsq;
  esqR /= sumEsq;
  esqG = greenEsq;
  esqG /= sumEsq;
  esqB = blueEsq;
  esqB /= sumEsq;
  esqR *= 256;
  esqG *= 256;
  esqB *= 256;

  Serial.print("RGB Direito R: ");
  Serial.print((int)dirR);
  Serial.print(" G: ");
  Serial.print((int)dirG);
  Serial.print(" B: ");
  Serial.print((int)dirB);
  Serial.print("\t");

  Serial.print("RGB Esquerdo R: ");
  Serial.print((int)esqR);
  Serial.print(" G: ");
  Serial.print((int)esqG);
  Serial.print(" B: ");
  Serial.println((int)esqB);


  if ((dirR > 120 && dirG < 100 && dirB < 100) || (esqR > 120 && esqG < 100 && esqB < 100)) {
    Serial.println("Vermelho");
    //buzzerOn();
  } else if ((dirR < 100 && dirG > 110 && dirB < 100) || (esqR < 100 && esqG > 110 && esqB < 100)) {
    Serial.println("Verde");
    //buzzerOn();
  } else if ((dirR < 100 && dirG < 100 && dirB > 130) || (esqR < 100 && esqG < 100 && esqB > 130)) {
    Serial.println("Azul");
    //buzzerOn();
  } 
}
