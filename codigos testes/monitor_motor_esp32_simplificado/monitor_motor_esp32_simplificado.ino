/*
  ============================================================
  MONITOR DE MOTOR (CORRENTE + VIBRAÇÃO) – ESP32   (versão simplificada)
  ============================================================
  Ligações (conforme o diagrama):
    SCT-013-020 ........ GPIO34 (com divisor 10k+10k e capacitor 10 µF)
    ADXL345 + OLED ..... SDA = GPIO21, SCL = GPIO22
    Botão (módulo) ..... GPIO13 (OUT = HIGH ao apertar)
    Relé 2 canais ...... IN1 = GPIO17 (LED de alerta)
                         IN2 = GPIO16 (motor, contato NO)

  Bibliotecas: Adafruit SSD1306 + Adafruit GFX
  Placa: "ESP32 Dev Module" (core Arduino-ESP32 2.x ou 3.x)

  NÍVEIS (vale sempre o PIOR entre corrente e vibração):
    1 Normal  : I < 2,5 A    e  vib < 1,5 G
    2 Atenção : I >= 2,5 A   ou vib >= 1,5 G   -> LED aceso
    3 Crítico : I > 3,33 A   ou vib > 2,5 G    -> corta o motor

  MOTOR NO CONTATO NO: relé ENERGIZADO = motor liberado.
  Se o ESP32 travar/reiniciar, o relé desliga e o motor para.

  COMO RELIGAR:
    - parou pelo botão ........ clicar o botão
    - parou por nível crítico . segurar o botão por 2 s
    - parou por falha ......... reiniciar o ESP32

  AVISO: isto é proteção em SOFTWARE. Não substitui um corte físico
  independente (ex.: botão de emergência NF em série com o motor).

  OBS. PLACA: em módulos ESP32-WROVER os GPIO16/17 são da PSRAM.
  Nesse caso troque PIN_RELE_LED / PIN_RELE_MOTOR (ex.: 25 e 26).
  ============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "driver/gpio.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ============================================================
//  CONFIGURAÇÕES
// ============================================================

// ---- Pinos ----
const int PIN_SDA        = 21;
const int PIN_SCL        = 22;
const int PIN_SCT        = 34;
const int PIN_BOTAO      = 13;
const int PIN_RELE_LED   = 17;   // IN1
const int PIN_RELE_MOTOR = 16;   // IN2

// ---- Polaridades ----
const bool RELE_ATIVO_BAIXO = true;   // IN em LOW liga o relé (módulo com optoacoplador)
const int  BOTAO_ATIVO      = HIGH;   // se o módulo der LOW ao apertar, troque para LOW

// ---- Limites ----
const float I_LIM_ATENCAO = 2.50f;    // A
const float I_LIM_CRITICO = 3.33f;    // A
const float V_LIM_ATENCAO = 1.50f;    // G
const float V_LIM_CRITICO = 2.50f;    // G
const float HISTERESE     = 0.10f;    // folga para sair de um nível (A ou G)

// ---- Tempos (ms) ----
const uint32_t T_ATENCAO_MS     = 500;    // nível 2 precisa durar isso
const uint32_t T_CRITICO_MS     = 1000;   // nível 3 precisa durar isso para cortar
const uint32_t T_PARTIDA_MS     = 1500;   // pico de partida não corta o motor
const float    I_PARTIDA_MIN    = 0.30f;  // A: acima disso considera que o motor partiu
const uint32_t T_BLOQ_REARME_MS = 1000;   // após parar, ignora o botão por 1 s
const uint32_t T_HOLD_CRITICO_MS = 2000;  // segurar o botão após parada crítica
const uint32_t T_DEBOUNCE_MS    = 50;
const uint32_t T_CALIB_MAX_MS   = 25000;

// ---- Comportamento ----
const bool ARMAR_AUTOMATICAMENTE_NO_BOOT = false;  // false: só libera o motor ao apertar o botão

// ---- Wi-Fi (rede própria do ESP32) ----
const char* AP_SSID  = "Monitor-Motor";
const char* AP_SENHA = "motor1234";

// ---- Corrente (SCT-013-020: 20 A RMS -> 1 V RMS) ----
// Amostra a 1000 Hz (1 por tick do FreeRTOS). 250 amostras = 15 ciclos de 60 Hz.
const int   N_BLOCO_I      = 250;
const int   SOBREAMOSTRAS  = 4;        // leituras do ADC médias por amostra
const float A_POR_V        = 20.0f;    // A (RMS) por Volt (RMS)
const float FATOR_CAL      = 1.00f;    // ajuste fino comparando com um alicate amperímetro
const float ALFA_I         = 0.5f;     // suavização (0..1, menor = mais suave)
const float CORTE_I        = 0.05f;    // abaixo disso mostra 0
const float I_SATURADO     = 30.0f;    // valor assumido se o ADC saturar
const float ADC_DC_MIN     = 1000.0f;  // mV: offset fora da faixa = divisor/sensor com defeito
const float ADC_DC_MAX     = 2300.0f;
const float ADC_CLIP_BAIXO = 60.0f;    // mV
const float ADC_CLIP_ALTO  = 3150.0f;  // mV

// ---- Vibração (ADXL345 a 400 Hz com FIFO) ----
const int   N_BLOCO_V   = 100;     // 100 amostras @400 Hz = 250 ms
const float ALFA_DC_V   = 0.01f;   // remove a gravidade
const float ALFA_V      = 0.5f;
const float CORTE_V     = 0.02f;   // G
const float V_SATURADO  = 16.0f;

// ---- Calibração (motor DESLIGADO ao ligar o ESP32) ----
const int   BLOCOS_DESCARTE   = 2;
const int   BLOCOS_RUIDO      = 8;
const float CALIB_RUIDO_MAX_I = 0.30f;   // A
const float CALIB_RUIDO_MAX_V = 0.10f;   // G

// ---- Relé colado ----
const float    I_RELE_COLADO    = 0.50f;
const uint32_t T_ESPERA_RELE_MS = 3000;
const uint32_t T_RELE_COLADO_MS = 2000;

// ---- Watchdog: se o loop travar por mais que isso, o ESP32 reinicia ----
const uint32_t WDT_S = 3;

// ============================================================
//  ESTADO (escrito por uma task e lido pelas outras)
// ============================================================
enum { EST_CALIB = 0, EST_PARADO = 1, EST_OPER = 2 };
enum { M_NENHUM = 0, M_BOTAO, M_CRITICO, M_FALHA, M_CALIB, M_RELE };  // >= M_FALHA: só reiniciando

volatile int      g_estado = EST_CALIB;
volatile int      g_motivo = M_NENHUM;
volatile int      g_nivel = 1;
volatile bool     g_confirmando = false;   // nível 3 sendo confirmado (antes do corte)
volatile float    g_corrente = 0, g_vib = 0;
volatile uint32_t g_tsI = 0, g_tsV = 0;    // instante da última medida válida
volatile int      g_calibI = 0, g_calibV = 0;   // 0 = em andamento, 1 = ok, 2 = inválida
volatile bool     g_falhaI = false, g_falhaV = false;

uint32_t g_tParada = 0;
bool     g_adxlOk = false, g_displayOk = false, g_resetAnormal = false;
uint8_t  g_adxlAddr = 0x53;
char     g_ip[16] = "0.0.0.0";

Adafruit_SSD1306 display(128, 64, &Wire, -1);
WebServer server(80);

// ============================================================
//  RELÉS
// ============================================================
inline int nivelRele(bool ligado) { return (ligado ^ RELE_ATIVO_BAIXO) ? HIGH : LOW; }

void releInit(int pin) {
  gpio_set_level((gpio_num_t)pin, nivelRele(false));   // grava "desligado" antes de virar saída
  pinMode(pin, OUTPUT);
  gpio_set_level((gpio_num_t)pin, nivelRele(false));
}
inline void releLiga(int pin, bool ligado) { digitalWrite(pin, nivelRele(ligado)); }

// ============================================================
//  CORRENTE (task própria, 1 amostra por ms)
// ============================================================
void taskCorrente(void*) {
  TickType_t tk = xTaskGetTickCount();
  double soma = 0, somaq = 0, ruidoAcum = 0;
  int n = 0, blocos = 0, nRuido = 0, ruins = 0, perdas = 0;
  bool clip = false;
  float ruido = 0, iFilt = 0;

  for (;;) {
    if (xTaskDelayUntil(&tk, 1) == pdFALSE) perdas++;    // task atrasou: amostra perdida

    uint32_t acc = 0;
    for (int i = 0; i < SOBREAMOSTRAS; i++) acc += analogReadMilliVolts(PIN_SCT);
    float x = (float)acc / SOBREAMOSTRAS;

    if (x < ADC_CLIP_BAIXO || x > ADC_CLIP_ALTO) clip = true;
    soma += x; somaq += (double)x * x; n++;
    if (n < N_BLOCO_I) continue;

    // ---------- fim de bloco: RMS = desvio padrão (já elimina o offset DC) ----------
    double media = soma / n;
    double var = somaq / n - media * media;
    if (var < 0) var = 0;
    float irms = (float)sqrt(var) / 1000.0f * A_POR_V * FATOR_CAL;
    bool dcRuim = (media < ADC_DC_MIN || media > ADC_DC_MAX);
    bool valido = (perdas <= N_BLOCO_I / 100);            // até 1% de perda
    bool clipBloco = clip;
    n = 0; soma = 0; somaq = 0; perdas = 0; clip = false; blocos++;

    if (!valido) continue;                // descarta (sem atualizar g_tsI: o supervisor corta se persistir)
    ruins = dcRuim ? ruins + 1 : 0;
    if (clipBloco) irms = fmaxf(irms, I_SATURADO);
    uint32_t ts = millis();

    // ---- calibração: mede o ruído com o motor desligado ----
    if (g_calibI == 0) {
      if (ruins >= 2 || clipBloco) g_calibI = 2;
      else if (blocos > BLOCOS_DESCARTE) {
        ruidoAcum += (double)irms * irms;
        if (++nRuido >= BLOCOS_RUIDO) {
          ruido = (float)sqrt(ruidoAcum / nRuido);
          g_calibI = (ruido > CALIB_RUIDO_MAX_I) ? 2 : 1;   // ruído alto = motor ligado ou sinal instável
        }
      }
      g_tsI = ts;
      continue;
    }

    // ---- operação normal ----
    if (ruins >= 2) g_falhaI = true;
    float q = irms * irms - ruido * ruido;                  // tira o ruído medido
    float ic = (q > 0) ? sqrtf(q) : 0;
    iFilt += ALFA_I * (ic - iFilt);
    g_corrente = (iFilt < CORTE_I) ? 0.0f : iFilt;
    g_tsI = ts;
  }
}

// ============================================================
//  VIBRAÇÃO (ADXL345 via I2C, lido no loop())
// ============================================================
bool adxlEscreve(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(g_adxlAddr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool adxlLe(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(g_adxlAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)g_adxlAddr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool adxlInit() {
  const uint8_t enderecos[2] = {0x53, 0x1D};
  for (int a = 0; a < 2; a++) {
    g_adxlAddr = enderecos[a];
    uint8_t id = 0;
    if (!(adxlLe(0x00, &id, 1) && id == 0xE5)) continue;   // 0xE5 = ID do ADXL345

    bool ok = adxlEscreve(0x2D, 0x00);   // standby
    ok &= adxlEscreve(0x31, 0x0B);       // resolução total, ±16 g (256 LSB/g)
    ok &= adxlEscreve(0x2C, 0x0C);       // 400 Hz
    ok &= adxlEscreve(0x38, 0x00);       // FIFO bypass (limpa)
    ok &= adxlEscreve(0x38, 0x9F);       // FIFO stream
    ok &= adxlEscreve(0x2D, 0x08);       // inicia medição
    if (!ok) return false;

    delay(100);                          // confere se a FIFO está enchendo
    uint8_t st = 0;
    return adxlLe(0x39, &st, 1) && (st & 0x3F) >= 5;
  }
  return false;
}

struct ProcV {
  float  dc[3] = {0, 0, 0};
  float  filt = 0, ruido = 0;
  bool   init = false, clip = false;
  double somaq = 0, ruidoAcum = 0;
  int    n = 0, blocos = 0, nRuido = 0, ruins = 0;
} pv;

void processaAmostraV(const int16_t r[3]) {
  float a[3];
  for (int i = 0; i < 3; i++) a[i] = r[i] * (1.0f / 256.0f);
  if (!pv.init) { for (int i = 0; i < 3; i++) pv.dc[i] = a[i]; pv.init = true; }

  float s = 0;
  for (int i = 0; i < 3; i++) {
    if (fabsf(a[i]) > 15.5f) pv.clip = true;
    pv.dc[i] += ALFA_DC_V * (a[i] - pv.dc[i]);     // dc = gravidade
    float d = a[i] - pv.dc[i];
    s += d * d;
  }
  pv.somaq += s;                                    // |aceleração dinâmica|²
  if (++pv.n < N_BLOCO_V) return;

  // ---------- fim de bloco ----------
  float rms = (float)sqrt(pv.somaq / pv.n);
  float g = sqrtf(pv.dc[0]*pv.dc[0] + pv.dc[1]*pv.dc[1] + pv.dc[2]*pv.dc[2]);
  bool gravRuim = (g < 0.7f || g > 1.3f);           // parado deve medir ~1 G
  pv.ruins = gravRuim ? pv.ruins + 1 : 0;
  bool clip = pv.clip;
  if (clip) rms = fmaxf(rms, V_SATURADO);
  pv.somaq = 0; pv.n = 0; pv.clip = false; pv.blocos++;

  if (g_calibV == 0) {
    if (pv.ruins >= 4 || clip) g_calibV = 2;
    else if (pv.blocos > BLOCOS_DESCARTE) {
      pv.ruidoAcum += (double)rms * rms;
      if (++pv.nRuido >= BLOCOS_RUIDO) {
        pv.ruido = (float)sqrt(pv.ruidoAcum / pv.nRuido);
        g_calibV = (pv.ruido > CALIB_RUIDO_MAX_V) ? 2 : 1;
      }
    }
    return;
  }

  if (pv.ruins >= 4) g_falhaV = true;
  float q = rms * rms - pv.ruido * pv.ruido;
  float vc = (q > 0) ? sqrtf(q) : 0;
  pv.filt += ALFA_V * (vc - pv.filt);
  g_vib = (pv.filt < CORTE_V) ? 0.0f : pv.filt;
}

// Esvazia a FIFO do ADXL345 (chamado a cada volta do loop; a FIFO guarda 80 ms)
void lerVibracao() {
  static int erros = 0;
  if (!g_adxlOk) return;

  uint8_t st;
  int e = -1;
  if (adxlLe(0x39, &st, 1)) {
    e = min((int)(st & 0x3F), 32);
    for (int k = 0; k < e; k++) {
      uint8_t b[6];
      if (!adxlLe(0x32, b, 6)) { e = -1; break; }
      const int16_t r[3] = {(int16_t)((b[1] << 8) | b[0]),
                            (int16_t)((b[3] << 8) | b[2]),
                            (int16_t)((b[5] << 8) | b[4])};
      processaAmostraV(r);
    }
  }
  if (e < 0) { if (++erros >= 3) g_falhaV = true; }   // I2C com problema
  else { erros = 0; if (e > 0) g_tsV = millis(); }
}

// ============================================================
//  DISPLAY
// ============================================================
const char* textoStatus() {
  if (g_estado == EST_CALIB) return "CALIBRANDO...";
  if (g_estado == EST_OPER) {
    if (g_confirmando) return "N3 CONFIRMANDO...";
    return (g_nivel == 2) ? "N2 ATENCAO" : "N1 NORMAL";
  }
  switch (g_motivo) {
    case M_BOTAO:   return "EMERGENCIA-DESLIGADO";
    case M_CRITICO: return "N3 CRITICO-DESLIGADO";
    case M_FALHA:   return "FALHA SENSOR-DESLIG.";
    case M_CALIB:   return "CALIBRACAO INVALIDA";
    case M_RELE:    return "RELE COLADO? REINICIE";
    default:        return "PRONTO-APERTE BOTAO";
  }
}

void atualizaDisplay() {
  Serial.printf("%.2f A  %.2f G\n", (float)g_corrente, (float)g_vib);
  if (!g_displayOk) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("IP "); display.print(g_ip);
  display.setTextSize(2);
  display.setCursor(0, 14); display.printf("%.2f A", (float)g_corrente);
  display.setCursor(0, 32); display.printf("%.2f G", (float)g_vib);
  display.setTextSize(1);
  display.setCursor(0, 55); display.print(textoStatus());
  display.display();
}

// ============================================================
//  SEGURANÇA / MÁQUINA DE ESTADOS
// ============================================================
int  nivI = 1, nivV = 1;
uint32_t t2 = 0, t3 = 0;            // início das condições de nível 2 / 3 (0 = inativo)
bool motorParado = true;
uint32_t tPartidaAte = 0;
bool relBase = false; uint32_t relIni = 0;

void pararMotor(int motivo) {
  releLiga(PIN_RELE_MOTOR, false);
  g_estado = EST_PARADO;
  g_motivo = motivo;
  g_tParada = millis();
  g_nivel = (motivo == M_CRITICO) ? 3 : 1;
  g_confirmando = false;
}

void rearmar() {
  nivI = nivV = 1; t2 = t3 = 0; motorParado = true; tPartidaAte = 0;
  g_nivel = 1; g_confirmando = false; g_motivo = M_NENHUM; g_estado = EST_OPER;
  releLiga(PIN_RELE_MOTOR, true);
}

// nível 1/2/3 de uma grandeza, com histerese para SAIR do nível
int classifica(float v, float l2, float l3, int atual) {
  float t3 = (atual >= 3) ? (l3 - HISTERESE) : l3;
  float t2 = (atual >= 2) ? (l2 - HISTERESE) : l2;
  if (v > t3)  return 3;
  if (v >= t2) return 2;
  return 1;
}

// botão com debounce: 'clique' = acabou de apertar; 'segurou' = completou 2 s apertado
void lerBotao(bool& clique, bool& segurou) {
  static bool bruto = false, estavel = false, segurouJa = false;
  static uint32_t tMud = 0, tPress = 0;
  uint32_t now = millis();
  bool lido = (digitalRead(PIN_BOTAO) == BOTAO_ATIVO);
  clique = segurou = false;

  if (lido != bruto) { bruto = lido; tMud = now; }
  if ((now - tMud) >= T_DEBOUNCE_MS && bruto != estavel) {
    estavel = bruto;
    if (estavel) { clique = true; tPress = now; segurouJa = false; }
  }
  if (estavel && !segurouJa && (now - tPress) >= T_HOLD_CRITICO_MS) { segurouJa = true; segurou = true; }
}

void maquina(uint32_t now) {
  bool clique, segurou;
  lerBotao(clique, segurou);
  bool falha = g_falhaI || g_falhaV;

  // ---------- CALIBRANDO (relé do motor desligado) ----------
  if (g_estado == EST_CALIB) {
    if (g_calibI == 2 || g_calibV == 2)      pararMotor(M_CALIB);
    else if (falha)                          pararMotor(M_FALHA);
    else if (g_calibI == 1 && g_calibV == 1) {
      if (ARMAR_AUTOMATICAMENTE_NO_BOOT && !g_resetAnormal) rearmar();
      else { g_estado = EST_PARADO; g_motivo = M_NENHUM; g_tParada = now; }
    }
    else if (now > T_CALIB_MAX_MS)           pararMotor(M_FALHA);
    return;
  }

  // ---------- PARADO ----------
  if (g_estado == EST_PARADO) {
    if (g_motivo >= M_FALHA) return;                       // só reiniciando
    if (falha) { pararMotor(M_FALHA); return; }            // sensor falhou: impede o rearme

    bool podeReligar = (now - g_tParada) >= T_BLOQ_REARME_MS;
    bool pedido = (g_motivo == M_CRITICO) ? segurou : clique;
    if (podeReligar && pedido) { rearmar(); return; }

    // corrente com o relé aberto = relé colado
    if ((now - g_tParada) > T_ESPERA_RELE_MS && g_corrente > I_RELE_COLADO) {
      if (!relBase) { relBase = true; relIni = now; }
      else if ((now - relIni) >= T_RELE_COLADO_MS) pararMotor(M_RELE);
    } else relBase = false;
    return;
  }

  // ---------- OPERANDO ----------
  if (clique) { pararMotor(M_BOTAO); return; }
  if (falha || (int32_t)(now - g_tsI) > 1500 || (int32_t)(now - g_tsV) > 1000) {
    pararMotor(M_FALHA);                                   // sensor inválido, parado ou sem dados
    return;
  }

  float I = g_corrente, V = g_vib;

  // janela de partida: o pico não corta o motor
  if (I < I_PARTIDA_MIN * 0.7f) motorParado = true;
  else if (motorParado && I >= I_PARTIDA_MIN) { motorParado = false; tPartidaAte = now + T_PARTIDA_MS; }
  bool emPartida = ((int32_t)(tPartidaAte - now) > 0);

  nivI = classifica(I, I_LIM_ATENCAO, I_LIM_CRITICO, nivI);
  nivV = classifica(V, V_LIM_ATENCAO, V_LIM_CRITICO, nivV);
  int cand = max(nivI, nivV);
  if (emPartida && cand > 2) cand = 2;

  t2 = (cand >= 2) ? (t2 ? t2 : now) : 0;
  t3 = (cand >= 3) ? (t3 ? t3 : now) : 0;

  if (t3 && (now - t3) >= T_CRITICO_MS) { pararMotor(M_CRITICO); return; }
  g_confirmando = (t3 != 0);
  g_nivel = t3 ? 3 : ((t2 && (now - t2) >= T_ATENCAO_MS) ? 2 : 1);
}

void atualizaSaidas() {
  // reaplicadas a cada volta (protege contra qualquer glitch)
  releLiga(PIN_RELE_MOTOR, g_estado == EST_OPER);
  bool led = (g_estado == EST_OPER && g_nivel >= 2) ||
             (g_estado == EST_PARADO && (g_motivo == M_CRITICO || g_motivo >= M_FALHA));
  releLiga(PIN_RELE_LED, led);
}

// ============================================================
//  PÁGINA WEB (somente leitura)
// ============================================================
const char PAGINA[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="pt-br"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Monitor do Motor</title>
<style>
body{font-family:system-ui,sans-serif;background:#10141a;color:#e8edf2;max-width:480px;margin:0 auto;padding:12px;text-align:center}
.card{background:#1a212b;border-radius:12px;padding:14px;margin:10px 0}
.big{font-size:40px;font-weight:700}.lbl{font-size:12px;opacity:.7}
#st{font-weight:700;padding:14px;border-radius:12px;background:#546e7a}
</style></head><body>
<h2>Monitor do Motor</h2>
<div id="st">...</div>
<div class="card"><div class="big"><span id="i">0.00</span> A</div><div class="lbl">Corrente</div></div>
<div class="card"><div class="big"><span id="v">0.00</span> G</div><div class="lbl">Vibração</div></div>
<script>
const cor=['#546e7a','#2a7d46','#b8860b','#c62828'];
async function atualiza(){try{
 const r=await(await fetch('/dados',{cache:'no-store'})).json();
 document.getElementById('i').textContent=r.i.toFixed(2);
 document.getElementById('v').textContent=r.v.toFixed(2);
 const s=document.getElementById('st');s.textContent=r.txt;
 s.style.background=r.est==0?cor[0]:(r.est==1?cor[3]:cor[r.niv]);
}catch(e){document.getElementById('st').textContent='SEM CONEXÃO';}}
atualiza();setInterval(atualiza,500);
</script></body></html>
)rawliteral";

void hRaiz() { server.send_P(200, "text/html", PAGINA); }

void hDados() {
  char buf[200];
  snprintf(buf, sizeof(buf), "{\"i\":%.2f,\"v\":%.2f,\"est\":%d,\"niv\":%d,\"txt\":\"%s\"}",
           (float)g_corrente, (float)g_vib, (int)g_estado, (int)g_nivel, textoStatus());
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void taskWeb(void*) {
  for (;;) { server.handleClient(); vTaskDelay(pdMS_TO_TICKS(2)); }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void iniciaWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg;
  cfg.timeout_ms = WDT_S * 1000;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_S, true);          // true = reinicia o ESP32 se travar
#endif
}

void setup() {
  Serial.begin(115200);

  // Relés SEMPRE começam desligados
  releInit(PIN_RELE_MOTOR);
  releInit(PIN_RELE_LED);
  pinMode(PIN_BOTAO, BOTAO_ATIVO == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);

  // Reset anormal (watchdog, queda de tensão, panic): nunca rearma sozinho
  esp_reset_reason_t rr = esp_reset_reason();
  g_resetAnormal = (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
                    rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SCT, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(50);                       // I2C nunca fica preso esperando

  g_displayOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Wire.setClock(400000);
  if (!g_displayOk) Serial.println("ERRO: display OLED nao encontrado");

  g_adxlOk = adxlInit();
  if (!g_adxlOk) { Serial.println("ERRO: ADXL345 - motor bloqueado"); g_falhaV = true; }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_SENHA);
  strncpy(g_ip, WiFi.softAPIP().toString().c_str(), sizeof(g_ip) - 1);
  server.on("/", hRaiz);
  server.on("/dados", hDados);
  server.begin();

  uint32_t t = millis();
  g_tsI = g_tsV = t; g_tParada = t;

  // Se alguma task não subir, marca falha: o motor nunca é liberado
  if (xTaskCreatePinnedToCore(taskCorrente, "corrente", 4096, NULL, 5, NULL, 1) != pdPASS) g_falhaI = true;
  if (xTaskCreatePinnedToCore(taskWeb,      "web",      8192, NULL, 1, NULL, 0) != pdPASS)
    Serial.println("AVISO: web nao iniciou");

  iniciaWatchdog();
  esp_task_wdt_add(NULL);                    // o loop() precisa "dar sinal de vida"
}

void loop() {
  esp_task_wdt_reset();

  lerVibracao();
  uint32_t now = millis();
  maquina(now);
  atualizaSaidas();

  static uint32_t tDisp = 0;
  if (now - tDisp >= 300) { tDisp = now; atualizaDisplay(); }

  vTaskDelay(pdMS_TO_TICKS(10));
}
