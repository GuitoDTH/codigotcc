/*
 * ============================================================
 *  SISTEMA DE MONITORAMENTO DE LIQUIDIFICADOR
 *  ESP32 - SCT-013-020 + ADXL345 + OLED + Relé 2CH + Botão
 * ============================================================
 *
 * PINAGEM:
 *  SCT-013-020  → GPIO34 (ADC1_CH6) - TIP | GND - RING
 *  ADXL345      → SDA:GPIO21 | SCL:GPIO22 | VCC:3.3V | GND
 *  OLED SSD1306 → SDA:GPIO21 | SCL:GPIO22 | VCC:3.3V | GND
 *  RELÉ IN1     → GPIO16 (LED alerta)
 *  RELÉ IN2     → GPIO17 (corte motor)
 *  BOTÃO EMERG. → GPIO13 (INPUT_PULLUP)
 *
 * BIBLIOTECAS NECESSÁRIAS (instalar via Library Manager):
 *  - Adafruit SSD1306
 *  - Adafruit GFX Library
 *  - Adafruit ADXL345
 *  - Adafruit Unified Sensor
 *  - AsyncTCP
 *  - ESPAsyncWebServer
 *
 * NÍVEIS DE OPERAÇÃO:
 *  Nível 1 - Normal   : I < 2.5A  E  Vib < 1.5G
 *  Nível 2 - Atenção  : 2.5A ≤ I ≤ 4.33A  OU  1.5G ≤ Vib ≤ 2.5G
 *  Nível 3 - Crítico  : I > 4.33A  OU  Vib > 2.5G  → desliga motor
 * ============================================================
 */

// ─── BIBLIOTECAS ────────────────────────────────────────────
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <math.h>

// ─── CONFIGURAÇÕES DE REDE (Access Point) ───────────────────
const char* AP_SSID     = "Monitor_Liquidificador";
const char* AP_PASSWORD = "esp32monitor";   // mínimo 8 caracteres
// IP padrão do AP: 192.168.4.1

// ─── PINOS ──────────────────────────────────────────────────
#define PIN_SCT       34    // ADC entrada do SCT-013-020
#define PIN_RELE_LED  16    // Relé IN1 → LED de alerta
#define PIN_RELE_MOTOR 17   // Relé IN2 → corte do motor
#define PIN_BOTAO     13    // Botão de emergência (pull-up interno)

// ─── DISPLAY OLED ───────────────────────────────────────────
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// ─── ADXL345 ────────────────────────────────────────────────
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ─── SERVIDOR WEB ───────────────────────────────────────────
AsyncWebServer server(80);

// ─── MUTEX para acesso seguro entre loop() e callbacks do server ──
portMUX_TYPE dadosMux = portMUX_INITIALIZER_UNLOCKED;

// ─── CALIBRAÇÃO DO SCT-013-020 ──────────────────────────────
/*
 * SCT-013-020: saída 0~1V proporcional a 0~20A (com burden resistor interno)
 * O ESP32 ADC lê 0~3.3V (12 bits = 0~4095)
 * O sinal AC é centrado em VCC/2 ≈ 1.65V via divisor resistivo externo
 * Fator de calibração: ajuste conforme medição real com multímetro
 */
#define ADC_BITS        4095.0f
#define ADC_VREF        3.3f
#define SCT_VOUT_MAX    1.0f    // tensão de pico máx do SCT (V)
#define SCT_IMAX        20.0f   // corrente máx do SCT (A)
#define SCT_CALIB       (SCT_IMAX / SCT_VOUT_MAX)  // A/V = 20
#define AMOSTRAS_RMS    500     // amostras por cálculo de RMS
#define OFFSET_ADC      2048    // ponto médio 12 bits (1.65V)

// ─── LIMIARES ───────────────────────────────────────────────
#define CORRENTE_ATENCAO   2.50f  // A
#define CORRENTE_CRITICO   4.33f  // A
#define VIBRACAO_ATENCAO   1.50f  // G
#define VIBRACAO_CRITICO   2.50f  // G

// ─── HISTÓRICO (2 minutos @ 1 amostra/s = 120 pontos) ───────
#define HIST_SIZE  120
struct DataPoint {
  float corrente;
  float vibracao;
  uint8_t nivel;
  uint32_t timestamp; // segundos desde boot
};
DataPoint historico[HIST_SIZE];
uint8_t histIdx     = 0;
bool    histCheio   = false;
uint32_t ultimoHist = 0;

// ─── VARIÁVEIS GLOBAIS ───────────────────────────────────────
volatile float g_corrente     = 0.0f;
volatile float g_vibracao     = 0.0f;
volatile uint8_t g_nivel      = 1;
volatile bool g_motorLigado   = true;   // estado comandado pelo web/botao
volatile bool g_emergencia    = false;
volatile bool g_motorCortado  = false;  // estado físico do relé

uint32_t ultimaAmostragem  = 0;
uint32_t ultimoDisplay     = 0;
bool     botaoAnterior     = HIGH;

// ─── PROTÓTIPOS ─────────────────────────────────────────────
float   lerCorrenteRMS();
float   lerVibracaoG();
uint8_t calcularNivel(float i, float v);
void    aplicarNivel(uint8_t nivel);
void    atualizarDisplay(float i, float v, uint8_t nivel);
void    salvarHistorico(float i, float v, uint8_t nivel);
void    gerarDadosJSON(char* buf, size_t len);
void    gerarHistoricoJSON(char* buf, size_t len);
void    configurarRotasWebServer();

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Monitor Liquidificador ESP32 ===");

  // ── Pinos ──
  pinMode(PIN_RELE_LED,   OUTPUT);
  pinMode(PIN_RELE_MOTOR, OUTPUT);
  pinMode(PIN_BOTAO,      INPUT_PULLUP);

  // Relés ativos em LOW (módulos comuns); motor ligado, LED apagado
  digitalWrite(PIN_RELE_LED,   HIGH);  // LED desligado
  digitalWrite(PIN_RELE_MOTOR, HIGH);  // Motor ligado (NC → COM)

  // ── ADC ──
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // 0~3.3V

  // ── I2C ──
  Wire.begin(21, 22);

  // ── OLED ──
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("[ERRO] OLED não encontrado!");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Iniciando...");
    display.display();
  }

  // ── ADXL345 ──
  if (!accel.begin()) {
    Serial.println("[ERRO] ADXL345 não encontrado!");
  } else {
    accel.setRange(ADXL345_RANGE_4_G);   // ±4G - adequado para vibração
    accel.setDataRate(ADXL345_DATARATE_200_HZ);
    Serial.println("[OK] ADXL345 inicializado");
  }

  // ── Wi-Fi Access Point ──
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("[WIFI] AP IP: ");
  Serial.println(ip);

  // ── Web Server ──
  configurarRotasWebServer();
  server.begin();
  Serial.println("[WEB] Servidor iniciado");
                                                                S
  // ── Display inicial com IP ──
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Monitor Liquid.");
  display.println("Conecte ao WiFi:");
  display.println(AP_SSID);
  display.print("IP: ");
  display.println(ip);
  display.display();
  delay(3000);
}

// ════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ════════════════════════════════════════════════════════════
void loop() {
  uint32_t agora = millis();

  // ── Leitura do botão de emergência (debounce simples) ──
  bool botaoAtual = digitalRead(PIN_BOTAO);
  if (botaoAnterior == HIGH && botaoAtual == LOW) {
    // borda de descida → botão pressionado
    portENTER_CRITICAL(&dadosMux);
    g_emergencia  = !g_emergencia;
    g_motorLigado = !g_emergencia;
    portEXIT_CRITICAL(&dadosMux);
    Serial.print("[BTN] Emergência: ");
    Serial.println(g_emergencia ? "ATIVADA" : "DESATIVADA");
    delay(50); // debounce
  }
  botaoAnterior = botaoAtual;

  // ── Amostragem principal (a cada 100ms → ~10 amostras/s para o loop,
  //    mas lerCorrenteRMS() faz 500 leituras internas em ~50ms) ──
  if (agora - ultimaAmostragem >= 100) {
    ultimaAmostragem = agora;

    float corrente = lerCorrenteRMS();
    float vibracao = lerVibracaoG();

    portENTER_CRITICAL(&dadosMux);
    g_corrente = corrente;
    g_vibracao = vibracao;
    portEXIT_CRITICAL(&dadosMux);

    // Determinar nível
    uint8_t nivel = calcularNivel(corrente, vibracao);
    portENTER_CRITICAL(&dadosMux);
    g_nivel = nivel;
    portEXIT_CRITICAL(&dadosMux);

    // Aplicar ações de relé
    aplicarNivel(nivel);

    Serial.printf("[DATA] I=%.3fA  Vib=%.3fG  Nível=%d  Motor=%s  Emerg=%s\n",
                  corrente, vibracao, nivel,
                  g_motorLigado ? "ON" : "OFF",
                  g_emergencia  ? "SIM" : "NAO");
  }

  // ── Histórico (1 ponto/segundo) ──
  if (agora - ultimoHist >= 1000) {
    ultimoHist = agora;
    salvarHistorico(g_corrente, g_vibracao, g_nivel);
  }

  // ── Atualizar display (a cada 500ms) ──
  if (agora - ultimoDisplay >= 500) {
    ultimoDisplay = agora;
    atualizarDisplay(g_corrente, g_vibracao, g_nivel);
  }
}

// ════════════════════════════════════════════════════════════
//  LEITURA DA CORRENTE RMS (SCT-013-020)
// ════════════════════════════════════════════════════════════
float lerCorrenteRMS() {
  /*
   * O SCT-013-020 possui burden resistor interno que entrega 0~1V para 0~20A.
   * O sinal AC é centrado em VCC/2 via dois resistores de 10kΩ no pino GPIO34.
   * Calculamos o RMS subtraindo o offset DC médio das amostras.
   */
  long soma    = 0;
  long somaSq  = 0;

  // Primeira passagem: calcular offset DC (média)
  for (int i = 0; i < AMOSTRAS_RMS; i++) {
    soma += analogRead(PIN_SCT);
    delayMicroseconds(50); // ~20kHz de amostragem → cobre bem 60Hz
  }
  long offset = soma / AMOSTRAS_RMS;

  // Segunda passagem: RMS com offset removido
  soma = 0;
  for (int i = 0; i < AMOSTRAS_RMS; i++) {
    long sample = analogRead(PIN_SCT) - offset;
    somaSq += sample * sample;
    delayMicroseconds(50);
  }

  float rmsADC   = sqrt((float)somaSq / AMOSTRAS_RMS);
  float rmsVolt  = (rmsADC / ADC_BITS) * ADC_VREF;
  float correnteA = rmsVolt * SCT_CALIB;

  // Filtro: descarta valores negativos ou muito pequenos (ruído)
  if (correnteA < 0.05f) correnteA = 0.0f;

  return correnteA;
}

// ════════════════════════════════════════════════════════════
//  LEITURA DA VIBRAÇÃO (ADXL345)
// ════════════════════════════════════════════════════════════
float lerVibracaoG() {
  /*
   * Calculamos a magnitude do vetor aceleração e subtraímos 1G (gravidade)
   * para obter apenas a componente dinâmica de vibração.
   */
  sensors_event_t evento;
  accel.getEvent(&evento);

  float ax = evento.acceleration.x / 9.81f; // m/s² → G
  float ay = evento.acceleration.y / 9.81f;
  float az = evento.acceleration.z / 9.81f;

  float magnitude = sqrt(ax*ax + ay*ay + az*az);

  // Remove componente estática da gravidade (≈1G)
  float vibracao = fabs(magnitude - 1.0f);
  if (vibracao < 0.0f) vibracao = 0.0f;

  return vibracao;
}

// ════════════════════════════════════════════════════════════
//  CALCULAR NÍVEL DE OPERAÇÃO
// ════════════════════════════════════════════════════════════
uint8_t calcularNivel(float i, float v) {
  if (g_emergencia) return 3; // botão de emergência → sempre crítico

  if (i > CORRENTE_CRITICO || v > VIBRACAO_CRITICO) {
    return 3; // CRÍTICO
  } else if (i > CORRENTE_ATENCAO || v > VIBRACAO_ATENCAO) {
    return 2; // ATENÇÃO
  } else {
    return 1; // NORMAL
  }
}

// ════════════════════════════════════════════════════════════
//  APLICAR NÍVEL (acionar relés)
// ════════════════════════════════════════════════════════════
void aplicarNivel(uint8_t nivel) {
  /*
   * Relés do módulo comum são ativos em LOW:
   *  HIGH → bobina desligada (contato NO aberto / NC fechado)
   *  LOW  → bobina energizada (contato NO fechado / NC aberto)
   *
   * RELÉ IN1 (LED): LOW = LED aceso
   * RELÉ IN2 (MOTOR): LOW = motor DESLIGADO (abre NO que alimenta motor)
   *   → ajuste conforme sua fiação (NO ou NC)
   */

  // Se o operador desligou pelo web server ou emergência
  if (!g_motorLigado || g_emergencia) {
    digitalWrite(PIN_RELE_LED,   LOW);   // LED alerta aceso
    digitalWrite(PIN_RELE_MOTOR, LOW);   // Motor desligado
    g_motorCortado = true;
    return;
  }

  switch (nivel) {
    case 1: // NORMAL
      digitalWrite(PIN_RELE_LED,   HIGH); // LED apagado
      digitalWrite(PIN_RELE_MOTOR, HIGH); // Motor ligado
      g_motorCortado = false;
      break;

    case 2: // ATENÇÃO
      digitalWrite(PIN_RELE_LED,   LOW);  // LED aceso
      digitalWrite(PIN_RELE_MOTOR, HIGH); // Motor ligado
      g_motorCortado = false;
      break;

    case 3: // CRÍTICO
      digitalWrite(PIN_RELE_LED,   LOW);  // LED aceso
      digitalWrite(PIN_RELE_MOTOR, LOW);  // Motor desligado
      g_motorCortado  = true;
      g_motorLigado   = false; // trava: só religar pelo web server
      break;
  }
}

// ════════════════════════════════════════════════════════════
//  ATUALIZAR DISPLAY OLED
// ════════════════════════════════════════════════════════════
void atualizarDisplay(float i, float v, uint8_t nivel) {
  IPAddress ip = WiFi.softAPIP();

  display.clearDisplay();

  // Título
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("== Monitor Liquid ==");

  // Corrente
  display.setCursor(0, 12);
  display.print("Corrente: ");
  display.print(i, 2);
  display.println(" A");

  // Vibração
  display.setCursor(0, 22);
  display.print("Vibracao: ");
  display.print(v, 2);
  display.println(" G");

  // Nível
  display.setCursor(0, 34);
  display.setTextSize(1);
  display.print("Nivel: ");
  switch (nivel) {
    case 1:
      display.print("1 - NORMAL");
      break;
    case 2:
      display.print("2 - ATENCAO");
      break;
    case 3:
      display.print("3 - CRITICO!");
      break;
  }

  // Status motor
  display.setCursor(0, 44);
  display.print("Motor: ");
  display.print(g_motorCortado ? "DESLIGADO" : "LIGADO");

  // IP
  display.setCursor(0, 54);
  display.print("IP: ");
  display.print(ip);

  display.display();
}

// ════════════════════════════════════════════════════════════
//  SALVAR HISTÓRICO
// ════════════════════════════════════════════════════════════
void salvarHistorico(float i, float v, uint8_t nivel) {
  historico[histIdx].corrente  = i;
  historico[histIdx].vibracao  = v;
  historico[histIdx].nivel     = nivel;
  historico[histIdx].timestamp = millis() / 1000;

  histIdx = (histIdx + 1) % HIST_SIZE;
  if (histIdx == 0) histCheio = true;
}

// ════════════════════════════════════════════════════════════
//  PÁGINA HTML – armazenada em flash (PROGMEM) para não consumir
//  heap na task do TCP e evitar o assert "Required to lock TCPIP"
// ════════════════════════════════════════════════════════════
static const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Monitor Liquidificador</title>
  <style>
    :root {
      --bg: #0d1117;
      --surface: #161b22;
      --border: #30363d;
      --text: #e6edf3;
      --muted: #8b949e;
      --green: #3fb950;
      --yellow: #d29922;
      --red: #f85149;
      --blue: #58a6ff;
      --radius: 10px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', system-ui, sans-serif;
      background: var(--bg);
      color: var(--text);
      min-height: 100vh;
      padding: 20px;
    }
    h1 {
      text-align: center;
      font-size: 1.4rem;
      color: var(--blue);
      margin-bottom: 4px;
      letter-spacing: 1px;
    }
    .subtitle {
      text-align: center;
      color: var(--muted);
      font-size: 0.8rem;
      margin-bottom: 20px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 14px;
      margin-bottom: 18px;
    }
    .card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 16px 14px;
      text-align: center;
    }
    .card .label {
      font-size: 0.72rem;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 6px;
    }
    .card .value {
      font-size: 2rem;
      font-weight: 700;
      line-height: 1;
    }
    .card .unit {
      font-size: 0.85rem;
      color: var(--muted);
      margin-top: 3px;
    }
    /* nível banner */
    .nivel-banner {
      border-radius: var(--radius);
      padding: 14px;
      text-align: center;
      font-size: 1.1rem;
      font-weight: 700;
      letter-spacing: 1px;
      margin-bottom: 18px;
      border: 1px solid var(--border);
    }
    .nivel-1 { background: #0d2618; color: var(--green); border-color: var(--green); }
    .nivel-2 { background: #2d1f00; color: var(--yellow); border-color: var(--yellow); }
    .nivel-3 { background: #2d0d0d; color: var(--red); border-color: var(--red); animation: pulseRed 1s infinite; }
    @keyframes pulseRed {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.7; }
    }
    /* controles */
    .controls {
      display: flex;
      gap: 12px;
      margin-bottom: 18px;
      flex-wrap: wrap;
    }
    .btn {
      flex: 1;
      min-width: 120px;
      padding: 12px 10px;
      border: none;
      border-radius: var(--radius);
      font-size: 0.95rem;
      font-weight: 600;
      cursor: pointer;
      transition: opacity 0.2s, transform 0.1s;
      letter-spacing: 0.5px;
    }
    .btn:active { transform: scale(0.97); }
    .btn-green  { background: var(--green); color: #000; }
    .btn-red    { background: var(--red);   color: #fff; }
    .btn:disabled { opacity: 0.4; cursor: not-allowed; }
    /* histórico */
    .hist-wrap {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 14px;
      margin-bottom: 14px;
    }
    .hist-title {
      font-size: 0.8rem;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 10px;
    }
    #chartCanvas {
      width: 100%;
      height: 160px;
      display: block;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.8rem;
    }
    th {
      color: var(--muted);
      text-align: left;
      padding: 4px 6px;
      border-bottom: 1px solid var(--border);
      font-weight: 400;
    }
    td {
      padding: 4px 6px;
      border-bottom: 1px solid #1c2128;
    }
    .n1 { color: var(--green); }
    .n2 { color: var(--yellow); }
    .n3 { color: var(--red); font-weight: 700; }
    .status-dot {
      display: inline-block;
      width: 9px; height: 9px;
      border-radius: 50%;
      margin-right: 6px;
      background: var(--green);
    }
    .status-dot.off { background: var(--red); }
    footer {
      text-align: center;
      color: var(--muted);
      font-size: 0.72rem;
      margin-top: 14px;
    }
  </style>
</head>
<body>

  <h1>⚡ Monitor Liquidificador</h1>
  <p class="subtitle">ESP32 &mdash; Sistema de Proteção em Tempo Real</p>

  <div class="grid">
    <div class="card">
      <div class="label">Corrente RMS</div>
      <div class="value" id="valCorrente" style="color:var(--blue)">--</div>
      <div class="unit">Ampères</div>
    </div>
    <div class="card">
      <div class="label">Vibração</div>
      <div class="value" id="valVibracao" style="color:var(--blue)">--</div>
      <div class="unit">G-force</div>
    </div>
    <div class="card">
      <div class="label">Status Motor</div>
      <div class="value" id="valMotor" style="font-size:1rem;line-height:2.5">--</div>
    </div>
  </div>

  <div class="nivel-banner nivel-1" id="nivelBanner">
    ⏳ Aguardando dados...
  </div>

  <div class="controls">
    <button class="btn btn-green" id="btnLigar"   onclick="controlarMotor(true)">▶ LIGAR MOTOR</button>
    <button class="btn btn-red"   id="btnDesligar" onclick="controlarMotor(false)">⏹ DESLIGAR MOTOR</button>
  </div>

  <div class="hist-wrap">
    <div class="hist-title">📈 Histórico – últimos 2 minutos</div>
    <canvas id="chartCanvas"></canvas>
  </div>

  <div class="hist-wrap">
    <div class="hist-title">📋 Tabela de Eventos</div>
    <table>
      <thead>
        <tr>
          <th>Tempo (s)</th>
          <th>Corrente (A)</th>
          <th>Vibração (G)</th>
          <th>Nível</th>
        </tr>
      </thead>
      <tbody id="tabelaHist"></tbody>
    </table>
  </div>

  <footer>Monitor Liquidificador &bull; ESP32 Access Point &bull; Auto-atualização: 1s</footer>

<script>
// ─── Canvas chart ──────────────────────────────────────────
const canvas = document.getElementById('chartCanvas');
const ctx    = canvas.getContext('2d');
let histData = [];

function desenharGrafico(dados) {
  const W = canvas.offsetWidth;
  const H = 160;
  canvas.width  = W;
  canvas.height = H;

  ctx.clearRect(0, 0, W, H);

  // Fundo
  ctx.fillStyle = '#161b22';
  ctx.fillRect(0, 0, W, H);

  if (!dados || dados.length < 2) return;

  const N = dados.length;
  const pad = { t: 10, b: 20, l: 42, r: 10 };
  const cw = W - pad.l - pad.r;
  const ch = H - pad.t - pad.b;

  // Eixo corrente: 0~6A; vibração: 0~4G
  const maxI = 6, maxV = 4;

  // Grid
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 0.5;
  for (let g = 0; g <= 4; g++) {
    const y = pad.t + ch - (g / 4) * ch;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(pad.l + cw, y); ctx.stroke();
  }

  // Linha corrente (azul)
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  dados.forEach((d, i) => {
    const x = pad.l + (i / (N - 1)) * cw;
    const y = pad.t + ch - (Math.min(d.corrente, maxI) / maxI) * ch;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Linha vibração (laranja)
  ctx.strokeStyle = '#f0883e';
  ctx.lineWidth = 2;
  ctx.beginPath();
  dados.forEach((d, i) => {
    const x = pad.l + (i / (N - 1)) * cw;
    const y = pad.t + ch - (Math.min(d.vibracao, maxV) / maxV) * ch;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Eixo Y labels
  ctx.fillStyle = '#8b949e';
  ctx.font = '10px sans-serif';
  ctx.textAlign = 'right';
  ctx.fillText('6A/4G', pad.l - 4, pad.t + 4);
  ctx.fillText('3A/2G', pad.l - 4, pad.t + ch / 2 + 4);
  ctx.fillText('0',     pad.l - 4, pad.t + ch + 4);

  // Legenda
  ctx.textAlign = 'left';
  ctx.fillStyle = '#58a6ff';
  ctx.fillText('▬ Corrente', pad.l, H - 4);
  ctx.fillStyle = '#f0883e';
  ctx.fillText('▬ Vibração', pad.l + 80, H - 4);
}

// ─── Atualizar dados em tempo real ────────────────────────
async function atualizarDados() {
  try {
    const r = await fetch('/dados');
    const d = await r.json();

    // Cards
    document.getElementById('valCorrente').textContent = d.corrente.toFixed(2);
    document.getElementById('valVibracao').textContent = d.vibracao.toFixed(2);

    const motorEl = document.getElementById('valMotor');
    if (d.motorLigado) {
      motorEl.innerHTML = '<span class="status-dot"></span>LIGADO';
      motorEl.style.color = 'var(--green)';
    } else {
      motorEl.innerHTML = '<span class="status-dot off"></span>DESLIGADO';
      motorEl.style.color = 'var(--red)';
    }

    // Banner nível
    const banner = document.getElementById('nivelBanner');
    const nivelTexto = ['', '✅ NÍVEL 1 — NORMAL', '⚠️ NÍVEL 2 — ATENÇÃO', '🚨 NÍVEL 3 — CRÍTICO'];
    banner.textContent = nivelTexto[d.nivel];
    banner.className = 'nivel-banner nivel-' + d.nivel;

  } catch(e) { console.warn('Erro /dados:', e); }
}

async function atualizarHistorico() {
  try {
    const r = await fetch('/historico');
    const d = await r.json();
    histData = d.dados;

    desenharGrafico(histData);

    // Tabela (últimos 15 pontos)
    const tbody = document.getElementById('tabelaHist');
    tbody.innerHTML = '';
    const slice = histData.slice(-15).reverse();
    slice.forEach(p => {
      const cls = ['','n1','n2','n3'][p.nivel];
      const nTexto = ['','Normal','Atenção','Crítico'][p.nivel];
      tbody.innerHTML += `<tr>
        <td>${p.timestamp}</td>
        <td>${p.corrente.toFixed(2)}</td>
        <td>${p.vibracao.toFixed(2)}</td>
        <td class="${cls}">${nTexto}</td>
      </tr>`;
    });
  } catch(e) { console.warn('Erro /historico:', e); }
}

async function controlarMotor(ligar) {
  try {
    await fetch('/controle?motor=' + (ligar ? '1' : '0'));
    await atualizarDados();
  } catch(e) { console.warn('Erro /controle:', e); }
}

// ─── Iniciar loops ─────────────────────────────────────────
setInterval(atualizarDados,     1000);
setInterval(atualizarHistorico, 2000);
atualizarDados();
atualizarHistorico();
</script>
</body>
</html>
)rawhtml";

// ════════════════════════════════════════════════════════════
//  GERAR JSON DE DADOS EM TEMPO REAL
//  Usa buffer estático para evitar alocação de heap na task TCP
// ════════════════════════════════════════════════════════════
void gerarDadosJSON(char* buf, size_t len) {
  // Leitura atômica das variáveis globais
  float    corrente, vibracao;
  uint8_t  nivel;
  bool     motorLigado, emergencia;

  portENTER_CRITICAL(&dadosMux);
  corrente    = g_corrente;
  vibracao    = g_vibracao;
  nivel       = g_nivel;
  motorLigado = g_motorLigado;
  emergencia  = g_emergencia;
  portEXIT_CRITICAL(&dadosMux);

  snprintf(buf, len,
    "{\"corrente\":%.3f,\"vibracao\":%.3f,\"nivel\":%d,"
    "\"motorLigado\":%s,\"emergencia\":%s,\"uptime\":%lu}",
    corrente, vibracao, nivel,
    motorLigado ? "true" : "false",
    emergencia  ? "true" : "false",
    millis() / 1000UL
  );
}

// ════════════════════════════════════════════════════════════
//  GERAR JSON DO HISTÓRICO
//  Buffer fixo de 8 KB — suficiente para 120 pontos (~65 bytes cada)
// ════════════════════════════════════════════════════════════
#define HIST_JSON_BUF 8192
void gerarHistoricoJSON(char* buf, size_t len) {
  // Cópia local do histórico protegida por mutex
  DataPoint snap[HIST_SIZE];
  uint8_t   snapIdx;
  bool      snapCheio;

  portENTER_CRITICAL(&dadosMux);
  memcpy(snap, historico, sizeof(historico));
  snapIdx   = histIdx;
  snapCheio = histCheio;
  portEXIT_CRITICAL(&dadosMux);

  uint8_t total  = snapCheio ? HIST_SIZE : snapIdx;
  uint8_t inicio = snapCheio ? snapIdx   : 0;

  size_t pos = 0;
  pos += snprintf(buf + pos, len - pos, "{\"dados\":[");

  for (uint8_t i = 0; i < total && pos < len - 80; i++) {
    uint8_t idx = (inicio + i) % HIST_SIZE;
    if (i > 0) buf[pos++] = ',';
    pos += snprintf(buf + pos, len - pos,
      "{\"corrente\":%.2f,\"vibracao\":%.2f,\"nivel\":%d,\"timestamp\":%lu}",
      snap[idx].corrente, snap[idx].vibracao,
      snap[idx].nivel,    (unsigned long)snap[idx].timestamp
    );
  }

  snprintf(buf + pos, len - pos, "]}");
}

// ════════════════════════════════════════════════════════════
//  CONFIGURAR ROTAS DO WEB SERVER
// ════════════════════════════════════════════════════════════
void configurarRotasWebServer() {

  // Página principal – enviada direto da flash (PROGMEM), sem alocar heap
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", PAGE_HTML);
  });

  // Dados em tempo real (JSON)
  server.on("/dados", HTTP_GET, [](AsyncWebServerRequest* req) {
    static char buf[256];
    gerarDadosJSON(buf, sizeof(buf));
    AsyncWebServerResponse* resp = req->beginResponse(200, "application/json", buf);
    resp->addHeader("Access-Control-Allow-Origin", "*");
    req->send(resp);
  });

  // Histórico (JSON)
  server.on("/historico", HTTP_GET, [](AsyncWebServerRequest* req) {
    static char buf[HIST_JSON_BUF];
    gerarHistoricoJSON(buf, sizeof(buf));
    AsyncWebServerResponse* resp = req->beginResponse(200, "application/json", buf);
    resp->addHeader("Access-Control-Allow-Origin", "*");
    req->send(resp);
  });

  // Controle do motor via web
  server.on("/controle", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("motor")) {
      String val = req->getParam("motor")->value();
      portENTER_CRITICAL(&dadosMux);
      if (val == "1") {
        g_motorLigado = true;
        g_emergencia  = false;
      } else {
        g_motorLigado = false;
      }
      portEXIT_CRITICAL(&dadosMux);
      Serial.printf("[WEB] Motor %s pelo web server\n", val == "1" ? "LIGADO" : "DESLIGADO");
    }
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // 404
  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Rota nao encontrada");
  });
}

/*
 * ============================================================
 *  NOTAS DE CALIBRAÇÃO
 * ============================================================
 *
 * 1) DIVISOR RESISTIVO PARA GPIO34:
 *    Dois resistores de 10kΩ entre 3.3V e GND com o ponto médio
 *    no GPIO34. O capacitor de 10µF entre o ponto médio e GND
 *    filtra ruído. O sinal do SCT entra via capacitor de 10µF
 *    (acoplamento AC) entre a saída do SCT e o ponto médio.
 *
 * 2) AJUSTE DE SCT_CALIB:
 *    Meça a corrente real com um alicate amperímetro e ajuste
 *    SCT_CALIB até os valores baterem. O valor padrão é 20 (A/V).
 *
 * 3) RELÉ ATIVO EM LOW:
 *    Os módulos de relé com optoacoplador comuns são ativados
 *    com sinal LOW. Se o seu módulo for ativo em HIGH, inverta
 *    a lógica nos digitalWrite() da função aplicarNivel().
 *
 * 4) ADXL345 - ENDEREÇO I2C:
 *    Se o pino SDO/ALT estiver em GND → endereço 0x53 (padrão)
 *    Se o pino SDO/ALT estiver em VCC → endereço 0x1D
 *    A biblioteca Adafruit detecta automaticamente.
 *
 * 5) REDE Wi-Fi:
 *    SSID: Monitor_Liquidificador
 *    Senha: esp32monitor
 *    IP:   192.168.4.1
 *    Acesse: http://192.168.4.1 no navegador
 * ============================================================
 */
