/*
  Teste do ADXL345 como sensor de vibração (I2C, sem bibliotecas externas)

  Ligações (Arduino Uno/Nano):
    ADXL345 VCC -> 3.3V (módulos GY-291 aceitam 5V, confira o seu)
    ADXL345 GND -> GND
    ADXL345 SDA -> A4
    ADXL345 SCL -> A5
    ADXL345 CS  -> VCC (obrigatório para modo I2C)
    ADXL345 SDO -> GND (endereço 0x53) ou VCC (endereço 0x1D)

  Como funciona:
    1. Lê X, Y, Z em g.
    2. Calcula o módulo total: |a| = sqrt(x² + y² + z²).
    3. Em repouso |a| ~ 1 g (gravidade), então a vibração é |a| - 1 g.
    4. Se a vibração > LIMIAR_G, considera que há vibração.
*/

#include <Wire.h>

// ---------- Configurações ----------
#define ADXL_ADDR       0x53      // 0x1D se SDO estiver em VCC
const float LIMIAR_G  = 0.15;     // Limiar de vibração (em g). Ajuste conforme o seu caso
const int   PIN_LED   = 13;       // LED indicador
const unsigned long TEMPO_SEGURA_MS = 500;  // Mantém o alarme ativo por este tempo após a última detecção
const int   N_MEDIA   = 4;        // Nº de amostras para média móvel simples (suaviza ruído)

// ---------- Registradores ----------
#define REG_DEVID       0x00
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32

// Resolução em modo Full-Res: ~3.9 mg/LSB (independente da faixa)
const float ESCALA_G = 0.0039;

unsigned long ultimaDeteccao = 0;
bool vibrando = false;

// ---------- Funções I2C ----------
void escreverReg(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission();
}

uint8_t lerReg(uint8_t reg) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool lerAceleracao(float &x, float &y, float &z) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)ADXL_ADDR, (uint8_t)6) != 6) return false;

  int16_t rx = Wire.read() | (Wire.read() << 8);
  int16_t ry = Wire.read() | (Wire.read() << 8);
  int16_t rz = Wire.read() | (Wire.read() << 8);

  x = rx * ESCALA_G;
  y = ry * ESCALA_G;
  z = rz * ESCALA_G;
  return true;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);

  uint8_t id = lerReg(REG_DEVID);
  if (id != 0xE5) {
    Serial.print(F("ADXL345 nao encontrado! DEVID lido: 0x"));
    Serial.println(id, HEX);
    while (true) {                 // trava e pisca o LED em caso de erro
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(200);
    }
  }

  escreverReg(REG_DATA_FORMAT, 0x09);  // Full-Res + faixa ±4g
  escreverReg(REG_BW_RATE, 0x0D);      // Taxa de saída 800 Hz
  escreverReg(REG_POWER_CTL, 0x08);    // Modo medição

  Serial.println(F("ADXL345 OK. Monitorando vibracao..."));
  Serial.print(F("Limiar: ")); Serial.print(LIMIAR_G, 3); Serial.println(F(" g"));
  Serial.println(F("total_g\tvibracao_g\tstatus"));
}

// ---------- Loop ----------
void loop() {
  float somaTotal = 0;
  int validas = 0;

  // Média curta para reduzir ruído
  for (int i = 0; i < N_MEDIA; i++) {
    float x, y, z;
    if (lerAceleracao(x, y, z)) {
      somaTotal += sqrt(x * x + y * y + z * z);
      validas++;
    }
    delay(2);
  }

  if (validas == 0) {
    Serial.println(F("Erro de leitura I2C"));
    delay(100);
    return;
  }

  float total = somaTotal / validas;       // módulo total em g
  float vibracao = fabs(total - 1.0);      // remove a gravidade (~1 g)

  unsigned long agora = millis();

  if (vibracao > LIMIAR_G) {
    ultimaDeteccao = agora;
    vibrando = true;
  } else if (vibrando && (agora - ultimaDeteccao > TEMPO_SEGURA_MS)) {
    vibrando = false;
  }

  digitalWrite(PIN_LED, vibrando ? HIGH : LOW);

  Serial.print(total, 3);
  Serial.print('\t');
  Serial.print(vibracao, 3);
  Serial.print('\t');
  Serial.println(vibrando ? F("VIBRACAO!") : F("estavel"));

  delay(20);
}
