/*
  ============================================================
  Monitor de Corrente com SCT-013-020 + ESP32
  ============================================================
  - Leitura via ADC (GPIO34)
  - Filtro passa-alta adaptativo para remover o offset DC (~1,65V)
  - Cálculo de RMS por janela de amostras (várias ondas AC)
  - Calibração manual ou automática via comandos no Monitor Serial
  - Zona morta (deadband) para ignorar ruído quando não há carga

  Circuito:
    Tip (sinal)   -> GPIO34
    Sleeve (bias) -> ponto médio do divisor resistivo (2x 10k entre 3V3 e GND,
                     com capacitor 10uF entre o ponto médio e GND)

  Comandos no Monitor Serial (115200 baud):
    cal <valor>     -> define o fator de calibração manualmente (ex: cal 1.05)
    auto <A_real>   -> calibra automaticamente. Ligue uma carga com corrente
                        conhecida (medida com alicate amperímetro) e digite,
                        por exemplo: auto 2.50
    zero            -> refaz a calibração do offset DC (sem carga ligada)
  ============================================================
*/

#include <Arduino.h>

// ---------------- Configurações ----------------
const int   PINO_ADC            = 34;      // GPIO só-entrada, ideal para ADC
const float VREF                = 3.3;     // tensão de referência do ADC
const int   ADC_RES              = 4095;    // resolução 12 bits
const int   AMOSTRAS_POR_LEITURA = 2000;    // amostras por cálculo de RMS
const float RELACAO_SCT          = 20.0;    // SCT-013-020: 20A correspondem a 1V RMS na saída
const float ALPHA_OFFSET         = 0.0008;  // velocidade de adaptação do filtro de offset (bem lento)
const float CORRENTE_MINIMA      = 0.03;    // abaixo disso, considera ruído e zera (ajuste conforme seu ruído)
const int   ATRASO_AMOSTRAGEM_US = 150;     // intervalo entre amostras (define taxa de amostragem)

float fatorCalibracao = 1.0;                // multiplicador de calibração (ajustável)
float offsetDC;                              // valor médio (centro) do sinal, atualizado continuamente

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(PINO_ADC, ADC_11db); // permite ler até ~3,3V

  Serial.println();
  Serial.println("=== Monitor de Corrente SCT-013-020 ===");
  Serial.println("Comandos:");
  Serial.println("  cal <valor>    -> define fator de calibracao manualmente");
  Serial.println("  auto <A_real>  -> calibracao automatica com carga conhecida");
  Serial.println("  zero           -> refaz calibracao do offset (sem carga)");
  Serial.println();

  calibrarOffsetInicial();
}

void loop() {
  processarComandosSerial();

  float irms = calcularRMS();

  Serial.print("Irms: ");
  Serial.print(irms, 3);
  Serial.print(" A   |  offset ADC: ");
  Serial.print(offsetDC, 1);
  Serial.print("   |  fator cal: ");
  Serial.println(fatorCalibracao, 4);

  delay(500);
}

// ------------------------------------------------------------
// Calibração inicial do offset DC (executar sem carga conectada)
// ------------------------------------------------------------
void calibrarOffsetInicial() {
  Serial.println("Calibrando offset...");
  offsetDC = analogRead(PINO_ADC); // chute inicial

  // Fase 1: convergência rápida (alpha alto) por ~2s
  for (int i = 0; i < 4000; i++) {
    int leitura = analogRead(PINO_ADC);
    offsetDC += 0.05 * (leitura - offsetDC); // alpha bem maior aqui
    delayMicroseconds(ATRASO_AMOSTRAGEM_US);
  }

  Serial.print("Offset estabilizado em: ");
  Serial.println(offsetDC, 1);
}

// ------------------------------------------------------------
// Calcula o valor RMS da corrente, com filtro passa-alta adaptativo
// para remover deriva do offset DC (equivalente ao que a EmonLib faz)
// ------------------------------------------------------------
float calcularRMS() {
  double somaQuadrados = 0;

  for (int i = 0; i < AMOSTRAS_POR_LEITURA; i++) {
    int leitura = analogRead(PINO_ADC);

    // Filtro passa-alta: o offset "persegue" lentamente o valor médio do
    // sinal, filtrando ruído de baixa frequência e deriva térmica do bias
    offsetDC += ALPHA_OFFSET * (leitura - offsetDC);

    float valorCentralizado = leitura - offsetDC;
    somaQuadrados += (double)valorCentralizado * valorCentralizado;

    delayMicroseconds(ATRASO_AMOSTRAGEM_US);
  }

  float rmsADC     = sqrt(somaQuadrados / AMOSTRAS_POR_LEITURA);
  float rmsTensao  = (rmsADC / ADC_RES) * VREF;
  float rmsCorrente = rmsTensao * RELACAO_SCT * fatorCalibracao;

  // Zona morta: ignora ruído residual quando não há corrente real
  if (rmsCorrente < CORRENTE_MINIMA) rmsCorrente = 0.0;

  return rmsCorrente;
}

// ------------------------------------------------------------
// Processa comandos digitados no Monitor Serial
// ------------------------------------------------------------
void processarComandosSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("cal ")) {
    float novoFator = cmd.substring(4).toFloat();
    if (novoFator > 0) {
      fatorCalibracao = novoFator;
      Serial.print("Fator de calibracao definido manualmente: ");
      Serial.println(fatorCalibracao, 4);
    } else {
      Serial.println("Valor invalido. Use: cal 1.05");
    }
  }
  else if (cmd.startsWith("auto ")) {
    float correnteReal = cmd.substring(5).toFloat();
    if (correnteReal > 0) {
      float correnteMedida = calcularRMS();
      float correnteBase = correnteMedida / fatorCalibracao; // valor "cru", sem o fator atual

      if (correnteBase > 0.001) {
        fatorCalibracao = correnteReal / correnteBase;
        Serial.print("Calibracao automatica concluida. Novo fator: ");
        Serial.println(fatorCalibracao, 4);
      } else {
        Serial.println("Erro: corrente medida muito baixa. Ligue a carga antes de calibrar.");
      }
    } else {
      Serial.println("Valor invalido. Use: auto 2.50");
    }
  }
  else if (cmd == "zero") {
    calibrarOffsetInicial();
  }
  else if (cmd.length() > 0) {
    Serial.println("Comando nao reconhecido.");
  }
}
