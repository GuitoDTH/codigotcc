#include <Wire.h>
#include <math.h>

// ─── ADXL345 (I2C direto, sem biblioteca) ───────────────────
#define SDA_PIN     21
#define SCL_PIN     22
uint8_t ADXL_ADDR = 0x53;

#define REG_DEVID       0x00
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32
#define SCALE_G         0.0039f

// ─── SCT-013 ────────────────────────────────────────────────
#define PIN_SCT   34
#define AMOSTRAS  500

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool readAccel(float &x, float &y, float &z) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADXL_ADDR, (uint8_t)6) != 6) return false;

  int16_t rx = Wire.read() | (Wire.read() << 8);
  int16_t ry = Wire.read() | (Wire.read() << 8);
  int16_t rz = Wire.read() | (Wire.read() << 8);

  x = rx * SCALE_G;
  y = ry * SCALE_G;
  z = rz * SCALE_G;
  return true;
}

float lerVibracaoG() {
  float x, y, z;
  if (!readAccel(x, y, z)) return -1.0f;
  float magnitude = sqrtf(x*x + y*y + z*z);
  float vibracao  = fabsf(magnitude - 1.0f);
  return vibracao;
}

float lerCorrenteRMS() {
  long soma  = 0;
  long somaSq = 0;

  for (int i = 0; i < AMOSTRAS; i++) {
    soma += analogRead(PIN_SCT);
    delayMicroseconds(50);
  }
  long offset = soma / AMOSTRAS;

  for (int i = 0; i < AMOSTRAS; i++) {
    long sample = analogRead(PIN_SCT) - offset;
    somaSq += sample * sample;
    delayMicroseconds(50);
  }

  float rmsADC   = sqrtf((float)somaSq / AMOSTRAS);
  float rmsVolt  = (rmsADC / 4095.0f) * 3.3f;
  float corrente = rmsVolt * 20.0f;

  if (corrente < 0.05f) corrente = 0.0f;
  return corrente;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Detectar ADXL
  uint8_t id = readReg(REG_DEVID);
  if (id != 0xE5) {
    ADXL_ADDR = 0x1D;
    id = readReg(REG_DEVID);
  }

  if (id != 0xE5) {
    Serial.println("[ERRO] ADXL345 nao encontrado!");
    while (true) delay(1000);
  }

  writeReg(REG_DATA_FORMAT, 0x08); // full resolution, ±2g
  writeReg(REG_BW_RATE, 0x0A);     // 100 Hz
  writeReg(REG_POWER_CTL, 0x08);   // modo medicao

  Serial.println("[OK] ADXL345 inicializado");
  Serial.println("[OK] SCT-013 no GPIO34");
  Serial.println("----------------------------");
  Serial.println("Vibracao (G) | Corrente (A)");
  Serial.println("----------------------------");
  delay(500);
}

void loop() {
  float vibracao = lerVibracaoG();
  float corrente = lerCorrenteRMS();

  if (vibracao < 0) {
    Serial.println("Falha na leitura do ADXL!");
  } else {
    Serial.print("Vibracao: ");
    Serial.print(vibracao, 3);
    Serial.print(" G     |     Corrente: ");
    Serial.print(corrente, 3);
    Serial.println(" A");
  }

  delay(500);
}