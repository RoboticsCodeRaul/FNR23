#include <AFMotor.h>
#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725softi2c.h"
#include <Ultrasonic.h>
//#include "DFRobot_TCS34725.h"

const int sdaDir = 20, sclDir = 21, sdaEsq = 44, sclEsq = 45;
const int redPin = 53, greenPin = 49, bluePin = 51;
const int buzzerPin = 25;
const int btnCalibra = 22, btnLiga = 23;
const int triggerEsq = A11, echoEsq = A10, triggerDir = 24, echoDir = A8;

//DFRobot_TCS34725 tcs = DFRobot_TCS34725(&Wire, TCS34725_ADDRESS, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725softi2c rgbDir = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, sdaDir, sclDir);
Adafruit_TCS34725softi2c rgbEsq = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, sdaEsq, sclEsq);

Ultrasonic ultraEsq(triggerEsq, echoEsq);
Ultrasonic ultraDir(triggerDir, echoDir);

AF_DCMotor frenteDir(1);
AF_DCMotor frenteEsq(2);
AF_DCMotor trasEsq(3);
AF_DCMotor trasDir(4);

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float distEsq, distDir;
long tempoEsq, tempoDir;
int calibra = HIGH, liga = HIGH;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(btnCalibra, INPUT_PULLUP);
  pinMode(btnLiga, INPUT_PULLUP);

  //Define o tipo de sensor segue linha (digital)
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 40, 38, 36, 34, 32, 30, 28, 26 }, SensorCount);
  qtr.setEmitterPin(42);


  frenteDir.run(RELEASE);
  frenteEsq.run(RELEASE);
  trasDir.run(RELEASE);
  trasEsq.run(RELEASE);

  while (!rgbDir.begin()) {
    Serial.println("O sensor RGB direito não foi encontrado ... verifica as ligações");
    delay(1000);
  }

  while (!rgbEsq.begin()) {
    Serial.println("O sensor RGB esquerdo não foi encontrado ... verifica as ligações");
    delay(1000);
  }

  //calibração dos motores
  //sensorCalibrate();
  while (calibra != LOW) {
    calibra = digitalRead(btnCalibra);
    turnBlue();
  }

  sensorCalibrate();

  while (liga != LOW) {
    liga = digitalRead(btnLiga);
  }

  ledOff();
}

void loop() {
  // put your main code here, to run repeatedly:

  //checkColor();
  //moveR(100, 100);
  lineFollower();

  //tone(buzzerPin, 2300);
  //delay(3000);
  //noTone(buzzerPin);
  //delay(3000);
  //distancia();
  //turnRed();
  //digitalWrite(buzzerPin, HIGH);  //Set PIN 8 feet as HIGH = 5 v
  //delay(2000);                    //Set the delay time，2000ms
  //digitalWrite(buzzerPin, LOW);   //Set PIN 8 feet for LOW = 0 v
  //delay(2000);
}


void turnRed() {
  analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

void turnGreen() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 0);
}

void turnBlue() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255);
}

void turnPink() {
  analogWrite(redPin, 255);
  analogWrite(greenPin,0);
  analogWrite(bluePin, 255);
}


void ledOff() {
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}

void distancia() {
  tempoEsq = ultraEsq.timing();
  distEsq = ultraEsq.convert(tempoEsq, Ultrasonic::CM);
  tempoDir = ultraDir.timing();
  distDir = ultraDir.convert(tempoDir, Ultrasonic::CM);

  Serial.print("Esquerda: ");
  Serial.print(distEsq);
  Serial.print(" Direita: ");
  Serial.println(distDir);
}

void moveF(int velA, int velB) {  // -> para o carro andar para a frente

  frenteEsq.setSpeed(velA);
  trasEsq.setSpeed(velA);

  frenteDir.setSpeed(velB);
  trasDir.setSpeed(velB);

  frenteEsq.run(FORWARD);
  trasEsq.run(FORWARD);

  frenteDir.run(FORWARD);
  trasDir.run(FORWARD);
}

void moveB(int velA, int velB) {  // -> para o carro andar para tras
  frenteEsq.setSpeed(velA);
  trasEsq.setSpeed(velA);

  frenteDir.setSpeed(velB);
  trasDir.setSpeed(velB);

  frenteEsq.run(BACKWARD);
  trasEsq.run(BACKWARD);

  frenteDir.run(BACKWARD);
  trasDir.run(BACKWARD);
}

void moveL(int velA, int velB) {  // -> para o carro andar para a esquerda
  frenteEsq.setSpeed(velA);
  trasEsq.setSpeed(velA);

  frenteDir.setSpeed(velB);
  trasDir.setSpeed(velB);

  frenteEsq.run(BACKWARD);
  trasEsq.run(BACKWARD);

  frenteDir.run(FORWARD);
  trasDir.run(FORWARD);
}

void moveR(int velA, int velB) {  // -> para o carro andar para a direita
  frenteEsq.setSpeed(velA);
  trasEsq.setSpeed(velA);

  frenteDir.setSpeed(velB);
  trasDir.setSpeed(velB);

  frenteEsq.run(FORWARD);
  trasEsq.run(FORWARD);

  frenteDir.run(BACKWARD);
  trasDir.run(BACKWARD);
}

void moveStop() {  // -> para o carro parar
  frenteEsq.setSpeed(0);
  trasEsq.setSpeed(0);

  frenteDir.setSpeed(0);
  trasDir.setSpeed(0);

  frenteEsq.run(RELEASE);
  trasEsq.run(RELEASE);

  frenteDir.run(RELEASE);
  trasDir.run(RELEASE);
}

void sensorCalibrate() {
  turnPink();
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  //Serial.println();
  delay(1000);
  turnGreen();
}

void lineFollower() {

  checkColor();
  uint16_t position = qtr.readLineBlack(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);


  if (position < 500)  //10000000
  {
    moveR(150, 150);
    Serial.println("Direita Acentuada ");
  } else if (position >= 500 && position < 1000)  //01000000
  {
    moveR(150, 140);
    Serial.println("Direita Intermedia ");
  } else if (position >= 1000 && position < 1500)  //001000
  {
    moveR(150, 130);
    Serial.println("Direita Ligeira ");
  } else if (position >= 1500 && position < 2000)  //001000
  {
    moveR(150, 120);
    Serial.println("Direita Ligeira ");
  } else if (position >= 2000 && position < 2500)  //001000
  {
    moveR(150, 110);
    Serial.println("Direita Ligeira ");
  } else if (position >= 2500 && position < 4500)  //000001
  {
    moveF(150, 150);
    Serial.println("Frente ");
  } else if (position >= 4500 && position < 5000)  //000010
  {
    moveL(110, 150);
    Serial.println("Esquerda Ligeira ");
  } else if (position >= 5000 && position < 5500)  //000100
  {
    moveL(120, 150);
    Serial.println("Esquerda Intermedia ");
  } else if (position >= 5500 && position < 6000)  //000100
  {
    moveL(130, 150);
    Serial.println("Esquerda Intermedia ");
  } else if (position >= 6000 && position < 6500)  //000100
  {
    moveL(140, 150);
    Serial.println("Esquerda Intermedia ");
  } else if (position > 6500)  //000100
  {
    moveL(150, 150);
    Serial.println("Esquerda Acentuada ");
  }
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
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
    buzzerOn();

  } else if ((dirR < 100 && dirG > 110 && dirB < 100) || (esqR < 100 && esqG > 110 && esqB < 100)) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
    buzzerOn();
  } else if ((dirR < 100 && dirG < 100 && dirB > 130) || (esqR < 100 && esqG < 100 && esqB > 130)) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 255);
    buzzerOn();
  } else {
    ledOff();
  }
}

void buzzerOn() {
  moveStop();
  digitalWrite(buzzerPin, HIGH);  //Set PIN 8 feet as HIGH = 5 v
  delay(3000);
  digitalWrite(buzzerPin, LOW);
  ledOff();
}
