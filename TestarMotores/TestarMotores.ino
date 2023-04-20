#include <AFMotor.h>

AF_DCMotor frenteDir(1);
AF_DCMotor frenteEsq(2);
AF_DCMotor trasEsq(3);
AF_DCMotor trasDir(4);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  frenteDir.run(RELEASE);
  frenteEsq.run(RELEASE);
  trasDir.run(RELEASE);
  trasEsq.run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:
  moveF(255, 255);
  // delay(5000);
  // moveB(255, 255);
  // delay(5000);
  // moveR(255, 255);
  // delay(5000);
  // moveL(255, 255);
  // delay(5000);
  // moveStop();
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