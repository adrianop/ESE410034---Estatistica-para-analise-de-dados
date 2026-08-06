/*
  Arduino Mega 2560 + MPU6050
  Trabalho Prático: Fusão Sensorial e Estimação de Pose com Filtro de Kalman

  Saída serial por linha:
      gyro_rate,acc_angle

  Unidades:
      gyro_rate -> graus/s no eixo X
      acc_angle -> graus (roll calculado pelo acelerômetro)

  Ligações I2C no Arduino Mega 2560:
      MPU6050 VCC -> 5V (para módulo GY-521; confira o seu módulo)
      MPU6050 GND -> GND
      MPU6050 SDA -> pino 20 (SDA)
      MPU6050 SCL -> pino 21 (SCL)
      MPU6050 INT -> não utilizado

  O Wire.begin() usa automaticamente os pinos 20 e 21 no Mega 2560.
*/

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const uint32_t SERIAL_BAUD = 115200;
const uint32_t SAMPLE_PERIOD_US = 20000UL;  // 50 Hz -> dt = 0,02 s

float gyroXOffset = 0.0f;
uint32_t nextSampleMicros = 0;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readWord(uint8_t reg, int16_t &value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
  if (received != 2 || Wire.available() < 2) {
    return false;
  }

  value = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

bool initializeMPU6050() {
  delay(100);

  // Acorda o sensor e seleciona o relógio interno padrão.
  if (!writeRegister(0x6B, 0x00)) return false;  // PWR_MGMT_1

  // Filtro digital passa-baixas (DLPF) para reduzir ruído.
  if (!writeRegister(0x1A, 0x03)) return false;  // CONFIG

  // Faixas padrão usadas nas conversões abaixo.
  if (!writeRegister(0x1B, 0x00)) return false;  // GYRO_CONFIG: +/-250 graus/s
  if (!writeRegister(0x1C, 0x00)) return false;  // ACCEL_CONFIG: +/-2 g

  delay(100);
  return true;
}

bool calibrateGyro() {
  const uint16_t sampleCount = 500;
  long sum = 0;
  int16_t gyroRaw = 0;

  Serial.println(F("CALIBRANDO_GIROSCOPIO"));

  for (uint16_t i = 0; i < sampleCount; i++) {
    if (!readWord(0x43, gyroRaw)) {  // GYRO_XOUT_H
      return false;
    }
    sum += gyroRaw;
    delay(3);
  }

  gyroXOffset = (float)sum / (float)sampleCount;
  Serial.println(F("CALIBRACAO_CONCLUIDA"));
  return true;
}

void stopWithError(const __FlashStringHelper *message) {
  Serial.println(message);
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(250);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  Wire.begin();          // Mega: SDA = 20, SCL = 21
  Wire.setClock(400000); // I2C Fast Mode

  if (!initializeMPU6050()) {
    stopWithError(F("ERRO_MPU6050_NAO_ENCONTRADO"));
  }

  if (!calibrateGyro()) {
    stopWithError(F("ERRO_CALIBRACAO_GIROSCOPIO"));
  }

  nextSampleMicros = micros();
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextSampleMicros) < 0) {
    return;
  }
  nextSampleMicros += SAMPLE_PERIOD_US;

  int16_t accXRaw = 0;
  int16_t accYRaw = 0;
  int16_t accZRaw = 0;
  int16_t gyroXRaw = 0;

  const bool ok =
      readWord(0x3B, accXRaw) &&
      readWord(0x3D, accYRaw) &&
      readWord(0x3F, accZRaw) &&
      readWord(0x43, gyroXRaw);

  if (!ok) {
    Serial.println(F("ERRO_LEITURA_IMU"));
    return;
  }

  // Escalas configuradas:
  // Acelerômetro +/-2 g: 16384 LSB/g
  // Giroscópio +/-250 graus/s: 131 LSB/(graus/s)
  const float ax = accXRaw / 16384.0f;
  const float ay = accYRaw / 16384.0f;
  const float az = accZRaw / 16384.0f;
  const float gyroX = (gyroXRaw - gyroXOffset) / 131.0f;

  // Roll em torno do eixo X.
  const float accAngle = atan2(ay, az) * 180.0f / PI;

  // Protocolo esperado pelo src/main_pratica.py
  Serial.print(gyroX, 4);
  Serial.print(',');
  Serial.println(accAngle, 4);

  (void)ax; // Mantido para facilitar uma futura extensão para pitch.
}
