// Carrega a biblioteca Servo
#include <Servo.h>

// Cria um objeto chamado servo1
Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  // Indica que o servo1 esta ligado ao pino 10
  servo1.attach(10);
  servo2.attach(9);
  servo3.attach(22);
  Serial.begin(9600);
}

void loop() {
  // Movimenta o servo
  servo1.write(90);
  servo2.write(90);
  //delay(2000);
  //servo3.write(100);
}