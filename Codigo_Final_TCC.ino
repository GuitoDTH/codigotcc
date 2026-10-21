/*Código Final TCC
bla bla bla
nn sei oq colocar aqui sksksksks
*/

// Bibliotecas Utilizadas
#include <Arduino.h> // funções do Arduino no esp32
#include <Wire.h> // Comunicação com o ADXL e o Display
#include <WiFi.h> // Wifi do esp32
#include <WebServer.h> // servidor do esp32
#include <Adafruit_SSD1306.h>// driver do display
#include <Adafruit_GFX.h>// melhor personalização do display
#include <esp_task_wdt.h> // se o sistema acabar travando, é reiniciado
#include <esp_system.h> // mostra pq reiniciou
#include "driver/gpio.h" // controle de baixo nivel dos pinos

// Pinos
#define pino_i2c_dado = 21; //pino de dados da com-i2c
#define pino_i2c_clock = 22; //pino clock da i2c
#define pino_sct = 34; //pino do sensor de corrente
#define pino_bot = 13; //botão de emergência
#define pino_rele_led = 17; //pino pra acionar o led de alerta via relé
#define pino_rele_mot = 16; //pino pra acionar o motor de desligamento via relé

// ADXL
uint8_t ADXL_ADDR = 0x53;
#define REG_DEVID       0x00
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32
#define SCALE_G 0.0039f