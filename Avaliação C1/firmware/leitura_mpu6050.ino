/*
  Leitura da MPU6050 e envio das amostras pela Serial.

  Ligacoes I2C:
  - Arduino Mega 2560: SDA->20 e SCL->21.
  - Arduino Uno: SDA->A4 e SCL->A5.
  Use a tensao indicada no modulo MPU6050 (o CI sem regulador opera em 3,3 V).
  Saida a 50 Hz, 115200 baud: t_ms,ax,ay,az,gx,gy,gz (g e graus/s).
*/

#include <Wire.h>

// Registradores da MPU6050
const uint8_t ENDERECO_MPU     = 0x68;
const uint8_t REG_WHO_AM_I     = 0x75;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// Escalas para os fundos de +-2 g e +-250 graus/s
const float ESCALA_ACEL = 16384.0;
const float ESCALA_GIRO = 131.0;

const unsigned long PERIODO_MS = 20;

unsigned long proximaAmostra = 0;


// Acesso ao sensor
void escrever(uint8_t registrador, uint8_t valor) {
  Wire.beginTransmission(ENDERECO_MPU);
  Wire.write(registrador);
  Wire.write(valor);
  Wire.endTransmission(true);
}


uint8_t ler(uint8_t registrador) {
  Wire.beginTransmission(ENDERECO_MPU);
  Wire.write(registrador);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ENDERECO_MPU, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0;
}


bool sensorRespondeu() {
  Wire.beginTransmission(ENDERECO_MPU);
  return Wire.endTransmission() == 0;
}


// Os dois bytes vao em linhas separadas para a ordem de leitura nao inverter
int16_t lerPar() {
  uint8_t alto = Wire.read();
  uint8_t baixo = Wire.read();
  return (int16_t)(((uint16_t)alto << 8) | baixo);
}


// Programa principal
void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!sensorRespondeu()) {
    Serial.println(F("MPU6050 nao respondeu no I2C."));
    Serial.println(F("Confira alimentacao e I2C (Mega: SDA 20/SCL 21; Uno: A4/A5)."));
    delay(3000);
  }

  Serial.print(F("# WHO_AM_I = 0x"));
  Serial.println(ler(REG_WHO_AM_I), HEX);

  escrever(REG_PWR_MGMT_1, 0x00);    // sai do modo de repouso
  escrever(REG_GYRO_CONFIG, 0x00);   // +-250 graus/s
  escrever(REG_ACCEL_CONFIG, 0x00);  // +-2 g

  // o vicio de zero do giroscopio fica por conta do filtro
  Serial.println(F("# t_ms,ax,ay,az,gx,gy,gz"));

  proximaAmostra = millis();
}


void loop() {
  if ((long)(millis() - proximaAmostra) < 0) {
    return;
  }
  proximaAmostra += PERIODO_MS;
  unsigned long agora = millis();

  Wire.beginTransmission(ENDERECO_MPU);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  // 14 bytes seguidos: aceleracao, temperatura e giroscopio
  if (Wire.requestFrom((uint8_t)ENDERECO_MPU, (uint8_t)14, (uint8_t)true) != 14) {
    return;
  }

  int16_t acelX = lerPar();
  int16_t acelY = lerPar();
  int16_t acelZ = lerPar();
  lerPar();
  int16_t giroX = lerPar();
  int16_t giroY = lerPar();
  int16_t giroZ = lerPar();

  Serial.print(agora);                    Serial.print(',');
  Serial.print(acelX / ESCALA_ACEL, 4);   Serial.print(',');
  Serial.print(acelY / ESCALA_ACEL, 4);   Serial.print(',');
  Serial.print(acelZ / ESCALA_ACEL, 4);   Serial.print(',');
  Serial.print(giroX / ESCALA_GIRO, 3);   Serial.print(',');
  Serial.print(giroY / ESCALA_GIRO, 3);   Serial.print(',');
  Serial.println(giroZ / ESCALA_GIRO, 3);
}
