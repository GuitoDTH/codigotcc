/*
  ADXL345 + SCT-013-020 no ESP32 (sem bibliotecas externas)

  Ligações (conforme seu diagrama):
    ADXL345 SDA -> GPIO21 | SCL -> GPIO22 | VCC -> 3.3V | GND -> GND
    SCT-013 (jack): Ring -> GPIO34, Tip -> ponto de bias (divisor 10K/10K + 10uF)

  SCT-013-020 = 20A : 1V (saída em tensão, já tem resistor de carga interno)
  Serial Monitor em 115200 baud.
*/

#include <Wire.h>

// ---------- Pinos ----------
#define SDA_PIN 21
#define SCL_PIN 22
#define SCT_PIN 34

// ---------- ADXL345 ----------
uint8_t ADXL_ADDR = 0x53;
#define REG_DEVID       0x00
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32
#define SCALE_G 0.0039f

// ---------- SCT-013 ----------
#define SCT_AMPS_PER_VOLT 20.0f   // SCT-013-020: 20A -> 1V
#define SCT_WINDOW_MS     200     // 200 ms = 12 ciclos de 60 Hz (10 de 50 Hz)
#define SCT_NOISE_FLOOR_A 0.05f   // abaixo disso considera 0 A

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

// Mede corrente RMS (A). Também devolve o bias (offset DC) em mV para diagnóstico.
float readCurrentRMS(float &biasMv) {
  double sum = 0, sumSq = 0;
  uint32_t n = 0;
  uint32_t t0 = millis();

  while (millis() - t0 < SCT_WINDOW_MS) {
    double mv = analogReadMilliVolts(SCT_PIN);
    sum += mv;
    sumSq += mv * mv;
    n++;
  }

  double mean = sum / n;                       // componente DC (bias ~1650 mV)
  double var = (sumSq / n) - (mean * mean);    // variância = potência AC
  if (var < 0) var = 0;

  biasMv = mean;
  float vrms = sqrt(var) / 1000.0;             // Vrms em volts
  float irms = vrms * SCT_AMPS_PER_VOLT;
  return (irms < SCT_NOISE_FLOOR_A) ? 0.0f : irms;
}

bool adxlOk = false;

bool initADXL() {
  uint8_t id = 0xFF;
  const uint8_t enderecos[] = {0x53, 0x1D};
  for (uint8_t a : enderecos) {
    ADXL_ADDR = a;
    id = readReg(REG_DEVID);
    if (id == 0xE5) break;
  }

  if (id != 0xE5) {
    Serial.printf("ADXL345 NÃO respondeu (DEVID=0x%02X). Dispositivos no I2C:\n", id);
    int n = 0;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { Serial.printf("  0x%02X\n", a); n++; }
    }
    if (!n) Serial.println("  nenhum (confira VCC/GND/SDA/SCL do ADXL)");
    return false;
  }

  Serial.printf("ADXL345 OK (0x%02X)\n", ADXL_ADDR);
  writeReg(REG_DATA_FORMAT, 0x08);  // full resolution, ±2g
  writeReg(REG_BW_RATE, 0x0A);      // 100 Hz
  writeReg(REG_POWER_CTL, 0x08);    // modo medição
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ADC do SCT-013
  analogReadResolution(12);
  analogSetPinAttenuation(SCT_PIN, ADC_11db);  // faixa até ~3.3V

  // I2C / ADXL345
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  adxlOk = initADXL();   // não trava mais se falhar
}

void loop() {
  float biasMv;
  float corrente = readCurrentRMS(biasMv);   // leva 200 ms

  if (!adxlOk) adxlOk = initADXL();   // tenta de novo a cada ciclo

  float x, y, z;
  bool okAccel = adxlOk && readAccel(x, y, z);

  Serial.printf("Corrente: %5.2f A (bias %4.0f mV) | ", corrente, biasMv);
  if (okAccel) {
    Serial.printf("X: %6.3f g | Y: %6.3f g | Z: %6.3f g\n", x, y, z);
  } else {
    Serial.println("falha leitura ADXL345");
  }

  delay(300);
}
