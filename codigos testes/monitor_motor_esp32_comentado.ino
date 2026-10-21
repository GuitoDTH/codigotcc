/*
  ============================================================
  MONITOR DE MOTOR (CORRENTE + VIBRAÇÃO) – ESP32   (versão COMENTADA)
  ============================================================
  Esta é a mesma lógica do arquivo "simplificado", só que com comentários
  explicando cada bloco e cada linha importante, para estudo.

  Ligações (conforme o diagrama):
    SCT-013-020 ........ GPIO34 (com divisor 10k+10k e capacitor 10 µF)
    ADXL345 + OLED ..... SDA = GPIO21, SCL = GPIO22
    Botão (módulo) ..... GPIO13 (OUT = HIGH ao apertar)
    Relé 2 canais ...... IN1 = GPIO17 (LED de alerta)
                         IN2 = GPIO16 (motor, contato NO)

  NÍVEIS (vale sempre o PIOR entre corrente e vibração):
    1 Normal  : I < 2,5 A    e  vib < 1,5 G
    2 Atenção : I >= 2,5 A   ou vib >= 1,5 G   -> LED aceso
    3 Crítico : I > 3,33 A   ou vib > 2,5 G    -> corta o motor

  MOTOR NO CONTATO NO: relé ENERGIZADO = motor liberado.

  COMO RELIGAR:
    - parou pelo botão ........ clicar o botão
    - parou por nível crítico . segurar o botão por 2 s
    - parou por falha ......... reiniciar o ESP32

  AVISO: isto é proteção em SOFTWARE. Não substitui um corte físico
  independente (ex.: botão de emergência NF em série com o motor).
  ============================================================
*/

// ------------------------------------------------------------
//  BIBLIOTECAS
// ------------------------------------------------------------
#include <Arduino.h>            // funções básicas: pinMode, digitalWrite, millis, delay...
#include <Wire.h>               // comunicação I2C (ADXL345 e display)
#include <WiFi.h>               // Wi-Fi do ESP32
#include <WebServer.h>          // servidor web simples
#include <Adafruit_GFX.h>       // desenho de texto/formas no display
#include <Adafruit_SSD1306.h>   // driver do display OLED SSD1306
#include <esp_task_wdt.h>       // watchdog: reinicia o ESP32 se o programa travar
#include <esp_system.h>         // esp_reset_reason(): por que o ESP32 reiniciou
#include "driver/gpio.h"        // gpio_set_level(): controle de baixo nível dos pinos

// Se o core não informar a versão, assume a 2.x (só importa para o watchdog)
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ============================================================
//  CONFIGURAÇÕES (tudo que você pode querer ajustar)
// ============================================================

// ---- Pinos ----
const int PIN_SDA        = 21;   // fio de dados do I2C
const int PIN_SCL        = 22;   // fio de clock do I2C
const int PIN_SCT        = 34;   // entrada analógica do sensor de corrente
const int PIN_BOTAO      = 13;   // botão de liga/emergência
const int PIN_RELE_LED   = 17;   // IN1 do módulo de relé -> LED de alerta
const int PIN_RELE_MOTOR = 16;   // IN2 do módulo de relé -> motor

// ---- Polaridades ----
const bool RELE_ATIVO_BAIXO = true;   // true: o relé liga quando o pino vai a LOW
const int  BOTAO_ATIVO      = HIGH;   // nível do pino quando o botão é apertado

// ---- Limites ----
const float I_LIM_ATENCAO = 2.50f;    // A: a partir daqui é nível 2
const float I_LIM_CRITICO = 3.33f;    // A: acima daqui é nível 3
const float V_LIM_ATENCAO = 1.50f;    // G: a partir daqui é nível 2
const float V_LIM_CRITICO = 2.50f;    // G: acima daqui é nível 3
const float HISTERESE     = 0.10f;    // folga para SAIR de um nível (evita ficar oscilando)

// ---- Tempos (em milissegundos) ----
const uint32_t T_ATENCAO_MS      = 500;    // nível 2 precisa durar isso para valer
const uint32_t T_CRITICO_MS      = 1000;   // nível 3 precisa durar isso para cortar o motor
const uint32_t T_PARTIDA_MS      = 1500;   // após o motor partir, o pico não corta por este tempo
const float    I_PARTIDA_MIN     = 0.30f;  // A: corrente a partir da qual o motor "partiu"
const uint32_t T_BLOQ_REARME_MS  = 1000;   // após parar, o botão é ignorado por este tempo
const uint32_t T_HOLD_CRITICO_MS = 2000;   // tempo de botão apertado para religar após parada crítica
const uint32_t T_DEBOUNCE_MS     = 50;     // filtro do "tremido" do botão
const uint32_t T_CALIB_MAX_MS    = 25000;  // se a calibração passar disso, dá falha

// ---- Comportamento ----
// false: só libera o motor quando você aperta o botão
// true : libera sozinho após a calibração (nunca após reset anormal)
const bool ARMAR_AUTOMATICAMENTE_NO_BOOT = false;

// ---- Wi-Fi (rede própria do ESP32) ----
const char* AP_SSID  = "Monitor-Motor";
const char* AP_SENHA = "motor1234";    // mínimo 8 caracteres

// ---- Corrente (SCT-013-020: 20 A RMS -> 1 V RMS) ----
// A task amostra a 1000 Hz (1 amostra por tick do FreeRTOS).
// 250 amostras = 250 ms = 15 ciclos inteiros de 60 Hz (RMS mais estável).
const int   N_BLOCO_I      = 250;
const int   SOBREAMOSTRAS  = 4;        // cada amostra = média de 4 leituras do ADC (menos ruído)
const float A_POR_V        = 20.0f;    // 20 A de corrente por 1 V lido
const float FATOR_CAL      = 1.00f;    // ajuste fino (compare com um alicate amperímetro)
const float ALFA_I         = 0.5f;     // suavização: 1 = sem suavizar, menor = mais lento
const float CORTE_I        = 0.05f;    // abaixo disso a corrente mostrada é 0
const float I_SATURADO     = 30.0f;    // A: valor assumido se o ADC saturar (força o corte)
const float ADC_DC_MIN     = 1000.0f;  // mV: offset do sinal deve ficar em ~1650 mV;
const float ADC_DC_MAX     = 2300.0f;  //     fora dessa faixa = divisor/sensor com defeito
const float ADC_CLIP_BAIXO = 60.0f;    // mV: abaixo disso o ADC está saturado embaixo
const float ADC_CLIP_ALTO  = 3150.0f;  // mV: acima disso o ADC está saturado em cima

// ---- Vibração (ADXL345 a 400 Hz com FIFO) ----
const int   N_BLOCO_V   = 100;     // 100 amostras a 400 Hz = 250 ms por bloco
const float ALFA_DC_V   = 0.01f;   // velocidade do filtro que separa a gravidade da vibração
const float ALFA_V      = 0.5f;    // suavização da vibração
const float CORTE_V     = 0.02f;   // G: abaixo disso a vibração mostrada é 0
const float V_SATURADO  = 16.0f;   // G: valor assumido se o sensor saturar

// ---- Calibração (motor DESLIGADO ao ligar o ESP32) ----
const int   BLOCOS_DESCARTE   = 2;       // blocos iniciais ignorados
const int   BLOCOS_RUIDO      = 8;       // blocos usados para medir o ruído
const float CALIB_RUIDO_MAX_I = 0.30f;   // A: ruído maior que isso = calibração inválida
const float CALIB_RUIDO_MAX_V = 0.10f;   // G: idem para vibração

// ---- Relé colado ----
const float    I_RELE_COLADO    = 0.50f; // A: corrente que não deveria existir com o relé aberto
const uint32_t T_ESPERA_RELE_MS = 3000;  // espera após parar antes de começar a checar
const uint32_t T_RELE_COLADO_MS = 2000;  // quanto tempo a corrente precisa persistir

// ---- Watchdog: se o loop() travar por mais que isso, o ESP32 reinicia ----
const uint32_t WDT_S = 3;

// ============================================================
//  ESTADO DO SISTEMA
// ============================================================
// Estados possíveis do sistema
enum { EST_CALIB = 0, EST_PARADO = 1, EST_OPER = 2 };

// Motivos de parada. Tudo a partir de M_FALHA só se resolve reiniciando o ESP32.
enum { M_NENHUM = 0, M_BOTAO, M_CRITICO, M_FALHA, M_CALIB, M_RELE };

// "volatile" = outra task pode mudar o valor a qualquer momento;
// o compilador não pode "guardar" o valor antigo.
volatile int      g_estado = EST_CALIB;   // estado atual
volatile int      g_motivo = M_NENHUM;    // por que está parado
volatile int      g_nivel = 1;            // nível atual (1, 2 ou 3)
volatile bool     g_confirmando = false;  // true = nível 3 detectado, aguardando 1 s para cortar
volatile float    g_corrente = 0;         // corrente medida (A)
volatile float    g_vib = 0;              // vibração medida (G)
volatile uint32_t g_tsI = 0, g_tsV = 0;   // "ts" = timestamp da última medida válida (sinal de vida)
volatile int      g_calibI = 0, g_calibV = 0;   // calibração: 0 = andando, 1 = ok, 2 = inválida
volatile bool     g_falhaI = false, g_falhaV = false;   // sensor com falha

uint32_t g_tParada = 0;        // instante em que o motor parou
bool     g_adxlOk = false;     // ADXL345 encontrado?
bool     g_displayOk = false;  // display encontrado?
bool     g_resetAnormal = false;   // último reset foi por watchdog/queda de tensão/erro?
uint8_t  g_adxlAddr = 0x53;    // endereço I2C do ADXL345 (0x53 ou 0x1D)
char     g_ip[16] = "0.0.0.0"; // IP em texto, para mostrar no display

Adafruit_SSD1306 display(128, 64, &Wire, -1);   // display 128x64, via I2C, sem pino de reset
WebServer server(80);                           // servidor web na porta 80

// ============================================================
//  RELÉS
// ============================================================

// Converte "ligado/desligado" no nível elétrico correto do pino.
// O símbolo ^ é XOR. Com módulo ativo-baixo:
//   ligado=true  -> true ^ true   = false -> LOW  (relé liga)
//   ligado=false -> false ^ true  = true  -> HIGH (relé desliga)
inline int nivelRele(bool ligado) { return (ligado ^ RELE_ATIVO_BAIXO) ? HIGH : LOW; }

// Configura um pino de relé já no estado DESLIGADO, sem pulso no boot.
void releInit(int pin) {
  gpio_set_level((gpio_num_t)pin, nivelRele(false));   // grava "desligado" ANTES de virar saída
  pinMode(pin, OUTPUT);                                // agora vira saída (já desligado)
  gpio_set_level((gpio_num_t)pin, nivelRele(false));   // garantia extra
}

// Atalho para ligar/desligar um relé: releLiga(PIN_RELE_MOTOR, true)
inline void releLiga(int pin, bool ligado) { digitalWrite(pin, nivelRele(ligado)); }

// ============================================================
//  CORRENTE (task própria, 1 amostra por ms)
// ============================================================
// O SCT-013 entrega uma senoide de 60 Hz sobre uma tensão de repouso (~1,65 V).
// O valor RMS (eficaz) dessa senoide é proporcional à corrente.
void taskCorrente(void*) {
  TickType_t tk = xTaskGetTickCount();      // referência de tempo para a pausa de 1 ms
  double soma = 0, somaq = 0, ruidoAcum = 0;   // acumuladores: soma de x, soma de x², soma do ruído
  int n = 0;           // amostras no bloco atual
  int blocos = 0;      // blocos já fechados desde o início
  int nRuido = 0;      // blocos usados na medição de ruído
  int ruins = 0;       // blocos seguidos com offset ruim
  int perdas = 0;      // amostras perdidas no bloco atual
  bool clip = false;   // ADC saturou neste bloco?
  float ruido = 0;     // ruído medido na calibração (A)
  float iFilt = 0;     // corrente suavizada

  for (;;) {   // laço infinito: toda task roda para sempre
    // Espera até completar 1 tick (1 ms) desde a última vez -> 1000 amostras por segundo.
    // Se a task já estava atrasada e não precisou esperar, devolve pdFALSE = amostra perdida.
    if (xTaskDelayUntil(&tk, 1) == pdFALSE) perdas++;

    // Lê o ADC 4 vezes (já em milivolts) e tira a média
    uint32_t acc = 0;
    for (int i = 0; i < SOBREAMOSTRAS; i++) acc += analogReadMilliVolts(PIN_SCT);
    float x = (float)acc / SOBREAMOSTRAS;

    // Sinal encostou nos extremos do ADC? Então está saturado (cortado)
    if (x < ADC_CLIP_BAIXO || x > ADC_CLIP_ALTO) clip = true;

    // Acumula x e x² (necessários para calcular o RMS no fim do bloco)
    soma += x;
    somaq += (double)x * x;
    n++;
    if (n < N_BLOCO_I) continue;   // bloco incompleto: volta ao início do laço

    // ---------- FIM DO BLOCO: calcula a corrente ----------
    double media = soma / n;                       // valor DC (~1650 mV)
    double var = somaq / n - media * media;        // variância = média dos quadrados - quadrado da média
    if (var < 0) var = 0;                          // arredondamento pode dar negativo minúsculo
    // raiz da variância = RMS só da parte AC (a senoide, sem o offset)
    // mV -> V (/1000), V -> A (* 20), e ajuste fino de calibração
    float irms = (float)sqrt(var) / 1000.0f * A_POR_V * FATOR_CAL;

    bool dcRuim = (media < ADC_DC_MIN || media > ADC_DC_MAX);   // offset fora da faixa?
    bool valido = (perdas <= N_BLOCO_I / 100);                  // aceita até 1% de amostras perdidas
    bool clipBloco = clip;                                      // guarda antes de zerar

    // Zera tudo para o próximo bloco
    n = 0; soma = 0; somaq = 0; perdas = 0; clip = false; blocos++;

    // Bloco com amostras perdidas é descartado SEM atualizar g_tsI.
    // Se isso persistir, o supervisor percebe sensor "mudo" e para o motor.
    if (!valido) continue;

    ruins = dcRuim ? ruins + 1 : 0;                       // conta blocos ruins SEGUIDOS
    if (clipBloco) irms = fmaxf(irms, I_SATURADO);        // saturou: assume no mínimo 30 A
    uint32_t ts = millis();                               // hora desta medida

    // ---- CALIBRAÇÃO: mede o ruído com o motor desligado ----
    if (g_calibI == 0) {
      if (ruins >= 2 || clipBloco) {
        g_calibI = 2;                                     // calibração inválida
      } else if (blocos > BLOCOS_DESCARTE) {              // ignora os primeiros blocos
        ruidoAcum += (double)irms * irms;                 // com motor desligado, irms é só ruído
        if (++nRuido >= BLOCOS_RUIDO) {                   // juntou blocos suficientes
          ruido = (float)sqrt(ruidoAcum / nRuido);        // ruído RMS médio
          g_calibI = (ruido > CALIB_RUIDO_MAX_I) ? 2 : 1; // muito ruído = motor ligado/sinal instável
        }
      }
      g_tsI = ts;     // sinal de vida
      continue;       // não calcula corrente enquanto calibra
    }

    // ---- OPERAÇÃO NORMAL ----
    if (ruins >= 2) g_falhaI = true;                      // offset ruim por 2 blocos: sensor/divisor defeituoso

    // Remove o ruído "em quadratura": real = raiz(medido² - ruído²)
    float q = irms * irms - ruido * ruido;
    float ic = (q > 0) ? sqrtf(q) : 0;

    iFilt += ALFA_I * (ic - iFilt);                       // suavização exponencial
    g_corrente = (iFilt < CORTE_I) ? 0.0f : iFilt;        // abaixo de 0,05 A mostra 0
    g_tsI = ts;                                           // sinal de vida
  }
}

// ============================================================
//  ADXL345 (vibração) via I2C
// ============================================================

// Escreve 'val' no registrador 'reg' do ADXL345. Devolve true se deu certo.
bool adxlEscreve(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(g_adxlAddr);   // começa a falar com o ADXL
  Wire.write(reg);                      // diz qual registrador
  Wire.write(val);                      // diz o valor
  return Wire.endTransmission() == 0;   // 0 = o chip confirmou
}

// Lê 'len' bytes a partir do registrador 'reg' para dentro de 'buf'.
bool adxlLe(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(g_adxlAddr);
  Wire.write(reg);                                        // de onde quero ler
  if (Wire.endTransmission(false) != 0) return false;     // false = pausa sem encerrar a conversa
  if (Wire.requestFrom((int)g_adxlAddr, (int)len) != len) return false;   // pede os bytes
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();                 // copia para buf
  return true;
}

// Procura e configura o ADXL345. Devolve true se está funcionando.
bool adxlInit() {
  const uint8_t enderecos[2] = {0x53, 0x1D};   // o endereço depende do pino SDO do módulo
  for (int a = 0; a < 2; a++) {
    g_adxlAddr = enderecos[a];
    uint8_t id = 0;
    // Registrador 0x00 = ID de fábrica, sempre 0xE5. Se não bater, tenta o outro endereço.
    if (!(adxlLe(0x00, &id, 1) && id == 0xE5)) continue;

    bool ok = adxlEscreve(0x2D, 0x00);   // 0x2D = controle de energia; 0x00 = standby (para configurar)
    ok &= adxlEscreve(0x31, 0x0B);       // 0x31 = formato: resolução total, faixa ±16 g (256 LSB por G)
    ok &= adxlEscreve(0x2C, 0x0C);       // 0x2C = taxa de amostragem: 0x0C = 400 Hz
    ok &= adxlEscreve(0x38, 0x00);       // 0x38 = FIFO: 0x00 limpa a fila
    ok &= adxlEscreve(0x38, 0x9F);       // 0x9F = modo "stream": guarda continuamente as últimas 32 amostras
    ok &= adxlEscreve(0x2D, 0x08);       // 0x08 = liga a medição
    if (!ok) return false;               // alguma escrita falhou

    delay(100);                          // espera um pouco para a FIFO encher
    uint8_t st = 0;
    // 0x39 = status da FIFO; os 6 bits baixos (& 0x3F) dizem quantas amostras há.
    // Se tem pelo menos 5, o sensor está mesmo medindo.
    return adxlLe(0x39, &st, 1) && (st & 0x3F) >= 5;
  }
  return false;   // não achou nenhum ADXL345
}

// Variáveis do processamento de vibração agrupadas numa struct.
struct ProcV {
  float  dc[3] = {0, 0, 0};    // estimativa da gravidade em cada eixo (X, Y, Z)
  float  filt = 0;             // vibração suavizada
  float  ruido = 0;            // ruído medido na calibração
  bool   init = false;         // já recebeu a primeira amostra?
  bool   clip = false;         // sensor saturou neste bloco?
  double somaq = 0;            // soma dos quadrados do bloco
  double ruidoAcum = 0;        // acumulador do ruído na calibração
  int    n = 0;                // amostras no bloco
  int    blocos = 0;           // blocos fechados
  int    nRuido = 0;           // blocos usados no ruído
  int    ruins = 0;            // blocos seguidos com gravidade fora do esperado
} pv;                          // 'pv' é a instância usada no programa

// Processa UMA amostra (X, Y, Z em valores brutos do sensor).
void processaAmostraV(const int16_t r[3]) {
  float a[3];
  for (int i = 0; i < 3; i++) a[i] = r[i] * (1.0f / 256.0f);   // bruto -> G (256 unidades = 1 G)

  // Na primeira amostra, assume que ela É a gravidade
  if (!pv.init) { for (int i = 0; i < 3; i++) pv.dc[i] = a[i]; pv.init = true; }

  float s = 0;   // soma dos quadrados dos 3 eixos (vibração)
  for (int i = 0; i < 3; i++) {
    if (fabsf(a[i]) > 15.5f) pv.clip = true;                 // perto de 16 G = saturado
    pv.dc[i] += ALFA_DC_V * (a[i] - pv.dc[i]);               // acompanha devagar a parte constante (gravidade)
    float d = a[i] - pv.dc[i];                               // medida - gravidade = só a vibração
    s += d * d;
  }
  pv.somaq += s;                                             // acumula |vibração|²
  if (++pv.n < N_BLOCO_V) return;                            // bloco incompleto: sai

  // ---------- FIM DO BLOCO ----------
  float rms = (float)sqrt(pv.somaq / pv.n);                  // vibração RMS do bloco

  // Módulo da gravidade estimada: parado, deve dar ~1 G em qualquer posição do sensor
  float g = sqrtf(pv.dc[0]*pv.dc[0] + pv.dc[1]*pv.dc[1] + pv.dc[2]*pv.dc[2]);
  bool gravRuim = (g < 0.7f || g > 1.3f);                    // fora disso = sensor ou escala com problema
  pv.ruins = gravRuim ? pv.ruins + 1 : 0;                    // conta blocos ruins SEGUIDOS

  bool clip = pv.clip;
  if (clip) rms = fmaxf(rms, V_SATURADO);                    // saturou: assume 16 G
  pv.somaq = 0; pv.n = 0; pv.clip = false; pv.blocos++;      // zera para o próximo bloco

  // ---- CALIBRAÇÃO ----
  if (g_calibV == 0) {
    if (pv.ruins >= 4 || clip) {
      g_calibV = 2;                                          // inválida
    } else if (pv.blocos > BLOCOS_DESCARTE) {
      pv.ruidoAcum += (double)rms * rms;
      if (++pv.nRuido >= BLOCOS_RUIDO) {
        pv.ruido = (float)sqrt(pv.ruidoAcum / pv.nRuido);
        g_calibV = (pv.ruido > CALIB_RUIDO_MAX_V) ? 2 : 1;   // vibrando durante a calibração = inválida
      }
    }
    return;
  }

  // ---- OPERAÇÃO NORMAL ----
  if (pv.ruins >= 4) g_falhaV = true;                        // gravidade fora do esperado por muito tempo
  float q = rms * rms - pv.ruido * pv.ruido;                 // tira o ruído em quadratura
  float vc = (q > 0) ? sqrtf(q) : 0;
  pv.filt += ALFA_V * (vc - pv.filt);                        // suavização
  g_vib = (pv.filt < CORTE_V) ? 0.0f : pv.filt;              // abaixo de 0,02 G mostra 0
}

// Esvazia a FIFO do ADXL345 e processa cada amostra. Chamada a cada volta do loop().
void lerVibracao() {
  static int erros = 0;        // 'static': mantém o valor entre chamadas
  if (!g_adxlOk) return;       // sensor nunca foi encontrado: não faz nada

  uint8_t st;
  int e = -1;                  // -1 = erro
  if (adxlLe(0x39, &st, 1)) {                  // lê quantas amostras há na FIFO
    e = min((int)(st & 0x3F), 32);             // no máximo 32
    for (int k = 0; k < e; k++) {
      uint8_t b[6];
      // Registrador 0x32: 6 bytes = X, Y, Z (2 bytes cada). Ler isso "tira" uma amostra da FIFO.
      if (!adxlLe(0x32, b, 6)) { e = -1; break; }
      // Cada eixo vem com o byte menos significativo primeiro: (b[1] << 8) | b[0]
      const int16_t r[3] = {(int16_t)((b[1] << 8) | b[0]),
                            (int16_t)((b[3] << 8) | b[2]),
                            (int16_t)((b[5] << 8) | b[4])};
      processaAmostraV(r);
    }
  }

  if (e < 0) {                                 // falha de I2C
    if (++erros >= 3) g_falhaV = true;         // 3 seguidas = sensor com problema
  } else {
    erros = 0;                                 // leu bem: zera a contagem de erros
    if (e > 0) g_tsV = millis();               // recebeu amostras: atualiza o sinal de vida
  }
}

// ============================================================
//  DISPLAY
// ============================================================

// Texto de status mostrado no display e na página web.
const char* textoStatus() {
  if (g_estado == EST_CALIB) return "CALIBRANDO...";
  if (g_estado == EST_OPER) {
    if (g_confirmando) return "N3 CONFIRMANDO...";
    return (g_nivel == 2) ? "N2 ATENCAO" : "N1 NORMAL";
  }
  // Parado: o texto depende do motivo
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
  Serial.printf("%.2f A  %.2f G\n", (float)g_corrente, (float)g_vib);   // também imprime no monitor serial
  if (!g_displayOk) return;

  display.clearDisplay();                    // limpa o desenho (na memória do ESP32)
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);                    // letra pequena
  display.setCursor(0, 0);                   // (x, y) em pixels
  display.print("IP "); display.print(g_ip);

  display.setTextSize(2);                    // letra grande
  display.setCursor(0, 14); display.printf("%.2f A", (float)g_corrente);
  display.setCursor(0, 32); display.printf("%.2f G", (float)g_vib);

  display.setTextSize(1);
  display.setCursor(0, 55); display.print(textoStatus());

  display.display();                         // AGORA envia o desenho para a tela de verdade
}

// ============================================================
//  SEGURANÇA / MÁQUINA DE ESTADOS
// ============================================================
int      nivI = 1, nivV = 1;       // nível atual da corrente e da vibração (para a histerese)
uint32_t t2 = 0, t3 = 0;           // quando começou a condição de nível 2 / 3 (0 = não está ocorrendo)
bool     motorParado = true;       // motor considerado parado (para detectar a partida)
uint32_t tPartidaAte = 0;          // até quando vai a janela de partida
bool     relBase = false;          // já começou a contar "corrente com relé aberto"?
uint32_t relIni = 0;               // quando começou essa contagem

// Corta o motor e registra o motivo.
void pararMotor(int motivo) {
  releLiga(PIN_RELE_MOTOR, false);                       // desliga o relé NA HORA
  g_estado = EST_PARADO;
  g_motivo = motivo;
  g_tParada = millis();                                  // anota quando parou
  g_nivel = (motivo == M_CRITICO) ? 3 : 1;               // nível 3 só permanece após parada crítica
  g_confirmando = false;
}

// Libera o motor.
void rearmar() {
  nivI = nivV = 1; t2 = t3 = 0; motorParado = true; tPartidaAte = 0;   // zera a lógica
  g_nivel = 1; g_confirmando = false; g_motivo = M_NENHUM;
  g_estado = EST_OPER;
  releLiga(PIN_RELE_MOTOR, true);                        // liga o relé do motor
}

// Decide o nível (1, 2 ou 3) de UMA grandeza.
// Histerese: se já está num nível, o limiar para PERMANECER nele é um pouco menor.
int classifica(float v, float l2, float l3, int atual) {
  float t3 = (atual >= 3) ? (l3 - HISTERESE) : l3;
  float t2 = (atual >= 2) ? (l2 - HISTERESE) : l2;
  if (v > t3)  return 3;    // crítico: estritamente acima do limite
  if (v >= t2) return 2;    // atenção: a partir do limite
  return 1;
}

// Lê o botão com anti-tremido.
//   clique  = true UMA vez quando o botão acabou de ser apertado
//   segurou = true UMA vez quando completou 2 s apertado
void lerBotao(bool& clique, bool& segurou) {
  static bool bruto = false, estavel = false, segurouJa = false;   // bruto = leitura crua; estavel = já filtrada
  static uint32_t tMud = 0, tPress = 0;   // quando a leitura mudou; quando foi apertado
  uint32_t now = millis();
  bool lido = (digitalRead(PIN_BOTAO) == BOTAO_ATIVO);   // true = apertado
  clique = segurou = false;

  // Leitura mudou? Anota o instante e espera ela se estabilizar
  if (lido != bruto) { bruto = lido; tMud = now; }

  // Só aceita a mudança depois de 50 ms estável
  if ((now - tMud) >= T_DEBOUNCE_MS && bruto != estavel) {
    estavel = bruto;
    if (estavel) { clique = true; tPress = now; segurouJa = false; }   // acabou de apertar
  }

  // Continua apertado por 2 s? Gera o evento "segurou" uma única vez
  if (estavel && !segurouJa && (now - tPress) >= T_HOLD_CRITICO_MS) {
    segurouJa = true;
    segurou = true;
  }
}

// O "cérebro": decide o que fazer conforme o estado. Chamada a cada volta do loop().
void maquina(uint32_t now) {
  bool clique, segurou;
  lerBotao(clique, segurou);
  bool falha = g_falhaI || g_falhaV;   // algum sensor falhou?

  // ---------- ESTADO: CALIBRANDO (relé do motor fica desligado) ----------
  if (g_estado == EST_CALIB) {
    if (g_calibI == 2 || g_calibV == 2)      pararMotor(M_CALIB);   // calibração inválida
    else if (falha)                          pararMotor(M_FALHA);   // sensor falhou
    else if (g_calibI == 1 && g_calibV == 1) {                      // os dois calibraram
      if (ARMAR_AUTOMATICAMENTE_NO_BOOT && !g_resetAnormal) {
        rearmar();                                                  // libera sozinho
      } else {
        g_estado = EST_PARADO; g_motivo = M_NENHUM; g_tParada = now;   // aguarda o botão
      }
    }
    else if (now > T_CALIB_MAX_MS)           pararMotor(M_FALHA);   // calibração demorou demais
    return;
  }

  // ---------- ESTADO: PARADO ----------
  if (g_estado == EST_PARADO) {
    if (g_motivo >= M_FALHA) return;                       // falha grave: só reiniciando
    if (falha) { pararMotor(M_FALHA); return; }            // sensor falhou: impede o rearme

    // Pode religar? Precisa ter passado 1 s desde a parada.
    bool podeReligar = (now - g_tParada) >= T_BLOQ_REARME_MS;
    // Após parada crítica: segurar o botão 2 s. Nos outros casos: um clique.
    bool pedido = (g_motivo == M_CRITICO) ? segurou : clique;
    if (podeReligar && pedido) { rearmar(); return; }

    // Detecção de relé colado: corrente passando com o relé aberto
    if ((now - g_tParada) > T_ESPERA_RELE_MS && g_corrente > I_RELE_COLADO) {
      if (!relBase) { relBase = true; relIni = now; }                       // começa a contar
      else if ((now - relIni) >= T_RELE_COLADO_MS) pararMotor(M_RELE);      // persistiu: relé colado
    } else {
      relBase = false;                                                      // corrente sumiu: zera
    }
    return;
  }

  // ---------- ESTADO: OPERANDO ----------
  if (clique) { pararMotor(M_BOTAO); return; }   // botão = emergência

  // Sensor com falha, ou sem dado novo há tempo demais: para por segurança.
  // O (int32_t) com sinal evita erro de conta se a outra task atualizar o tempo um instante depois.
  if (falha || (int32_t)(now - g_tsI) > 1500 || (int32_t)(now - g_tsV) > 1000) {
    pararMotor(M_FALHA);
    return;
  }

  float I = g_corrente, V = g_vib;   // copia os valores atuais

  // Detecção da partida do motor
  if (I < I_PARTIDA_MIN * 0.7f) {
    motorParado = true;                                   // corrente baixa: motor parado
  } else if (motorParado && I >= I_PARTIDA_MIN) {
    motorParado = false;                                  // acabou de partir
    tPartidaAte = now + T_PARTIDA_MS;                     // abre a janela de partida
  }
  bool emPartida = ((int32_t)(tPartidaAte - now) > 0);    // ainda dentro da janela?

  // Nível de cada grandeza, e o pior dos dois
  nivI = classifica(I, I_LIM_ATENCAO, I_LIM_CRITICO, nivI);
  nivV = classifica(V, V_LIM_ATENCAO, V_LIM_CRITICO, nivV);
  int cand = max(nivI, nivV);                             // "candidato" ao nível do sistema
  if (emPartida && cand > 2) cand = 2;                    // durante a partida, o pico não corta

  // Cronômetros: guardam quando a condição começou (ou 0 se não está ocorrendo)
  t2 = (cand >= 2) ? (t2 ? t2 : now) : 0;
  t3 = (cand >= 3) ? (t3 ? t3 : now) : 0;

  // Crítico durou 1 s? CORTA o motor.
  if (t3 && (now - t3) >= T_CRITICO_MS) { pararMotor(M_CRITICO); return; }

  // Senão, só atualiza o nível exibido
  g_confirmando = (t3 != 0);                              // crítico em confirmação
  g_nivel = t3 ? 3 : ((t2 && (now - t2) >= T_ATENCAO_MS) ? 2 : 1);
}

// Aplica o estado nos relés. Reaplicado a cada volta: erro momentâneo se corrige sozinho.
void atualizaSaidas() {
  releLiga(PIN_RELE_MOTOR, g_estado == EST_OPER);        // motor só liga em OPERANDO

  // LED acende em atenção/crítico durante a operação, ou após parada crítica/falha
  bool led = (g_estado == EST_OPER && g_nivel >= 2) ||
             (g_estado == EST_PARADO && (g_motivo == M_CRITICO || g_motivo >= M_FALHA));
  releLiga(PIN_RELE_LED, led);
}

// ============================================================
//  PÁGINA WEB (somente leitura)
// ============================================================
// R"rawliteral( ... )rawliteral" = texto "cru", permite usar aspas à vontade.
// PROGMEM = guardado na memória flash, não na RAM.
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
// cores: cinza (calibrando), verde (nível 1), amarelo (nível 2), vermelho (nível 3/parado)
const cor=['#546e7a','#2a7d46','#b8860b','#c62828'];
async function atualiza(){try{
 // pede os dados ao ESP32 (rota /dados) e converte o JSON
 const r=await(await fetch('/dados',{cache:'no-store'})).json();
 document.getElementById('i').textContent=r.i.toFixed(2);
 document.getElementById('v').textContent=r.v.toFixed(2);
 const s=document.getElementById('st');s.textContent=r.txt;
 // est==0 calibrando, est==1 parado (vermelho), senão usa a cor do nível
 s.style.background=r.est==0?cor[0]:(r.est==1?cor[3]:cor[r.niv]);
}catch(e){document.getElementById('st').textContent='SEM CONEXÃO';}}
atualiza();setInterval(atualiza,500);   // repete a cada 500 ms
</script></body></html>
)rawliteral";

// Quando alguém abre o IP no navegador: devolve a página.
void hRaiz() { server.send_P(200, "text/html", PAGINA); }

// Rota /dados: devolve os valores atuais em JSON.
void hDados() {
  char buf[200];
  snprintf(buf, sizeof(buf), "{\"i\":%.2f,\"v\":%.2f,\"est\":%d,\"niv\":%d,\"txt\":\"%s\"}",
           (float)g_corrente, (float)g_vib, (int)g_estado, (int)g_nivel, textoStatus());
  server.sendHeader("Cache-Control", "no-store");   // impede o navegador de reaproveitar resposta velha
  server.send(200, "application/json", buf);
}

// Task do servidor web: fica atendendo os pedidos para sempre.
void taskWeb(void*) {
  for (;;) {
    server.handleClient();             // atende quem estiver conectado
    vTaskDelay(pdMS_TO_TICKS(2));      // pausa de 2 ms para não monopolizar o processador
  }
}

// ============================================================
//  SETUP / LOOP
// ============================================================

// Configura o watchdog. A função muda entre as versões 2.x e 3.x do core.
void iniciaWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg;
  cfg.timeout_ms = WDT_S * 1000;     // tempo limite
  cfg.idle_core_mask = 0;            // não monitora as tasks ociosas
  cfg.trigger_panic = true;          // ao estourar, reinicia o ESP32
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_S, true);    // true = reinicia o ESP32 se travar
#endif
}

// setup() roda UMA vez, ao ligar.
void setup() {
  Serial.begin(115200);              // monitor serial

  // 1) Relés SEMPRE começam desligados
  releInit(PIN_RELE_MOTOR);
  releInit(PIN_RELE_LED);

  // 2) Botão com resistor interno que "puxa" o pino para o lado oposto do aperto
  pinMode(PIN_BOTAO, BOTAO_ATIVO == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);

  // 3) Descobre por que o ESP32 reiniciou. Reset por erro/watchdog/queda de tensão
  //    = "anormal": nesse caso o motor NUNCA arma sozinho.
  esp_reset_reason_t rr = esp_reset_reason();
  g_resetAnormal = (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
                    rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT);

  // 4) ADC: 12 bits (0..4095) e atenuação de 11 dB (lê até ~3,1 V)
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SCT, ADC_11db);

  // 5) I2C a 400 kHz, com timeout de 50 ms (nunca fica preso esperando um sensor)
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  // 6) Display no endereço 0x3C
  g_displayOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Wire.setClock(400000);             // a biblioteca pode ter alterado a velocidade
  if (!g_displayOk) Serial.println("ERRO: display OLED nao encontrado");

  // 7) ADXL345. Se falhar, marca falha: o motor nunca será liberado.
  g_adxlOk = adxlInit();
  if (!g_adxlOk) { Serial.println("ERRO: ADXL345 - motor bloqueado"); g_falhaV = true; }

  // 8) Wi-Fi: o ESP32 cria sua própria rede (Access Point)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_SENHA);
  strncpy(g_ip, WiFi.softAPIP().toString().c_str(), sizeof(g_ip) - 1);   // guarda o IP para o display

  // 9) Servidor web: liga cada endereço à sua função
  server.on("/", hRaiz);
  server.on("/dados", hDados);
  server.begin();

  // 10) Inicializa os "sinais de vida" com a hora atual, para o supervisor
  //     não achar que os sensores já estão mortos.
  uint32_t t = millis();
  g_tsI = g_tsV = t;
  g_tParada = t;

  // 11) Cria as tasks. Argumentos: função, nome, tamanho da pilha, parâmetro,
  //     prioridade (maior = mais importante), handle, núcleo do processador.
  //     Se a task de corrente não subir, marca falha: o motor nunca é liberado.
  if (xTaskCreatePinnedToCore(taskCorrente, "corrente", 4096, NULL, 5, NULL, 1) != pdPASS) g_falhaI = true;
  if (xTaskCreatePinnedToCore(taskWeb,      "web",      8192, NULL, 1, NULL, 0) != pdPASS)
    Serial.println("AVISO: web nao iniciou");

  // 12) Liga o watchdog e inscreve o loop() nele
  iniciaWatchdog();
  esp_task_wdt_add(NULL);            // NULL = "esta própria task" (o loop)
}

// loop() roda repetidamente, para sempre.
void loop() {
  esp_task_wdt_reset();              // "estou vivo": zera o cronômetro do watchdog

  lerVibracao();                     // esvazia a FIFO do ADXL e processa as amostras
  uint32_t now = millis();           // hora atual
  maquina(now);                      // decide estado, níveis e paradas
  atualizaSaidas();                  // aplica o resultado nos relés

  // A cada 300 ms atualiza o display (demora ~25 ms, não vale fazer toda volta)
  static uint32_t tDisp = 0;
  if (now - tDisp >= 300) { tDisp = now; atualizaDisplay(); }

  vTaskDelay(pdMS_TO_TICKS(10));     // pausa de 10 ms (curta o bastante para a FIFO do ADXL não estourar)
}
