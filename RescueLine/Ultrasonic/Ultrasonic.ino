#include <Ultrasonic.h>

#define TRIGGER_PIN 30
#define ECHO_PIN 32

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

void setup() {
  Serial.begin(9600);
}

void loop() {
  float distancia;
  long tempo = ultrasonic.timing();

  distancia = ultrasonic.convert(tempo, Ultrasonic::CM);
  
  Serial.print("Tempo: ");
  Serial.print(tempo);
  Serial.print(" Distancia: ");
  Serial.println(distancia);

}
