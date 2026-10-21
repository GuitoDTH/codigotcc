/*
  Teste do ADXL345 no ESP32 (I2C, sem bibliotecas externas)

  Ligações (conforme seu diagrama):
    ADXL345 SDA -> GPIO21
    ADXL345 SCL -> GPIO22
    ADXL345 VCC -> 3.3V
    ADXL345 GND -> GND

  Abra o Serial Monitor em 115200 baud.
*/

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

// Endereço I2C: 0x53 (pino SDO no GND) ou 0x1D (SDO no 3.3V)
uint8_t ADXL_ADDR = 0x53;

// Registradores
#define REG_DEVID       0x00  // deve retornar 0xE5
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32

// Resolução total (full-res): 3.9 mg/LSB
#define SCALE_G 0.0039f

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

// Varre o barramento I2C (mostra também o OLED, se estiver ligado)
void scanI2C() {
  Serial.println("Escaneando I2C...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Dispositivo em 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  Nenhum dispositivo encontrado!");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  scanI2C();

  // Tenta 0x53 e, se não achar, 0x1D
  uint8_t id = readReg(REG_DEVID);
  if (id != 0xE5) {
    ADXL_ADDR = 0x1D;
    id = readReg(REG_DEVID);
  }

  Serial.printf("Endereço usado: 0x%02X | DEVID lido: 0x%02X\n", ADXL_ADDR, id);
  if (id != 0xE5) {
    Serial.println("ERRO: ADXL345 não respondeu (esperado 0xE5). Verifique fios, 3.3V e GND.");
    while (true) delay(1000);
  }
  Serial.println("ADXL345 OK!");

  writeReg(REG_DATA_FORMAT, 0x08);  // full resolution, faixa ±2g
  writeReg(REG_BW_RATE, 0x0A);      // 100 Hz
  writeReg(REG_POWER_CTL, 0x08);    // modo medição
}

void loop() {
  float x, y, z;
  if (readAccel(x, y, z)) {
    float mag = sqrtf(x * x + y * y + z * z);
    Serial.printf("X: %6.3f g | Y: %6.3f g | Z: %6.3f g | |a|: %5.3f g\n", x, y, z, mag);
  } else {
    Serial.println("Falha na leitura I2C");
  }
  delay(200);
}
