#include <AFMotor.h>     //Adafruit Motor Shield Library. First you must download and install AFMotor library
#include <QTRSensors.h>  //Pololu QTR Sensor Library. First you must download and install QTRSensors library

//AF_DCMotor motor1(1, MOTOR12_1KHZ); //create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
//AF_DCMotor motor2(2, MOTOR12_1KHZ); //create motor #2 using M2 output on Motor Drive Shield, set to 1kHz PWM frequency

AF_DCMotor frenteDir(1);
AF_DCMotor frenteEsq(2);
AF_DCMotor trasEsq(3);
AF_DCMotor trasDir(4);

#define LIMITS 100.0
double kp = 10.0, ki = 0.5, kd = 25.0;  //PID control gains <> Ganhos do controlo PID
int vel = 90;                           //Max Speed <> Velocidade Máxima dos motores
int vCurve = 60;                        //Curve outside wheel max speed limit <> Limite de velocidade da roda exterior na curva

#define EMITTER_PIN 42  //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2

QTRSensors qtr;
const uint8_t NUM_SENSORS = 8;
uint16_t sensorValues[NUM_SENSORS];

void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 40, 38, 36, 34, 32, 30, 28, 26 }, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);
  // delay(1500);
  sensorCalibrate();
  //set_motors(0, 0);
}

int lastError = 0;
int last_proportional = 0;
int integral = 0;

void loop() {

  int line = 0;               // The line sensor value <> Valor da linha no sensor
  int lineRef = 0;            // Reference line value <> Valor de referência da linha
  int erroP = 0;              // Proportional error <> Erro proporcional
  static double erroI = 0.0;  // Integral error <> Erro Integral
  static int erroD = 0.0;     // Differential error <> Erro diferencial
  static int erroPAnt = 0;    // Previous proportional eror <> Erro proporcional anterior
  double output = 0.0;        // PID control output <> Resultado do controlo PID

  int velM1 = 0, velM2 = 0;  // Motor speeds <> Velocidade dos motores

  uint16_t position = qtr.readLineBlack(sensorValues);

  line = (int)((double)(position + 1) * 0.0285714) - 100;
  Serial.print(line);

  //line = one.readLine();  // Read the line sensor value -100 to +100 <> Leitura do valor da linha -100 a +100
  //Serial.print(" Line:");Serial.print(line);

  erroP = lineRef - line;    //Proportional error <> Erro proporcional
  erroD = erroP - erroPAnt;  //Differential error <> Erro diferencial
  output = kp * (double)erroP + ki * erroI + kd * (double)erroD;
  //Clean integral error if line value is zero or if line signal has changed
  //Limpar o erro integral se o valor da linha é zero ou se o sinal da linha mudou
  if (erroP * erroPAnt <= 0) erroI = 0.0;
  if (output > LIMITS) output = LIMITS;  //Limit the output value <> Limitar o valor de saída
  else if (output < -LIMITS) output = -LIMITS;
  else erroI += (double)erroP;  //Increment integral error if output is below limits <> Incrementa o erro integral se a saída está dentro dos limites
  erroPAnt = erroP;
  //Serial.print("  Out:"); Serial.print(output);

  //Limit motors maximum and minimum speed <> Limitar mínimos e máximos da velocidade dos motores
  velM1 = vel - (int)output;
  velM2 = vel + (int)output;
  if (velM1 < -1)
    velM1 = 0;  //Minimum speed -1 causes motor to brake <> Velocidade mínima -1 faz o motor travar
  if (velM2 < -1)
    velM2 = 0;
  if (velM1 > vel + vCurve)
    velM1 = vel + vCurve;  //Maximum speed limit <> Limite da velocidade máxima
  if (velM2 > vel + vCurve)
    velM2 = vel + vCurve;

  Serial.print("   M1:");
  Serial.print(velM1);
  Serial.print("   M2:");
  Serial.println(velM2);

  // // set motor speeds using the two motor speed variables above
  if (line < -80) {
    moveR(velM2, velM2 - 30);
  } else if (line >= -80 && line <= 80)
    moveF(velM2, velM1);
  else {
    moveL(velM1, velM1 - 30);
  }
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

void moveR(int velA, int velB) {  // -> para o carro andar para a frente

  frenteEsq.setSpeed(velA);
  trasEsq.setSpeed(velA);

  frenteDir.setSpeed(velB);
  trasDir.setSpeed(velB);

  frenteEsq.run(FORWARD);
  trasEsq.run(FORWARD);

  frenteDir.run(BACKWARD);
  trasDir.run(BACKWARD);
}

void moveL(int velA, int velB) {  // -> para o carro andar para a frente

  frenteEsq.setSpeed(velA);
  trasEsq.setSpeed(velA);

  frenteDir.setSpeed(velB);
  trasDir.setSpeed(velB);

  frenteEsq.run(BACKWARD);
  trasEsq.run(BACKWARD);

  frenteDir.run(FORWARD);
  trasDir.run(FORWARD);
}

void set_motors(int motor1speed, int motor2speed) {
  // if (motor1speed > M1_maksimum_speed) motor1speed = M1_maksimum_speed;
  // if (motor2speed > M2_maksimum_speed) motor2speed = M2_maksimum_speed;
  // if (motor1speed < 0) motor1speed = 0;
  // if (motor2speed < 0) motor2speed = 0;
  frenteEsq.setSpeed(motor1speed);
  trasEsq.setSpeed(motor1speed);
  frenteDir.setSpeed(motor2speed);
  trasDir.setSpeed(motor2speed);
  frenteEsq.run(FORWARD);
  trasEsq.run(FORWARD);
  frenteDir.run(FORWARD);
  trasDir.run(FORWARD);
}

void sensorCalibrate() {
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  //Serial.println();
  delay(1000);
}
