/*Código Final TCC
bla bla bla
nn sei oq colocar aqui sksksksks
*/

// Bibliotecas Utilizadas
#include <Arduino.h> // funções do Arduino no esp32
#include <Wire.h> // Comunicação com o ADXL e o Display
#include <Adafruit_ADXL345_U.h> //Biblioteca ADXL
#include <WiFi.h> // Wifi do esp32
#include <WebServer.h> // servidor do esp32
#include <Adafruit_SSD1306.h>// driver do display
#include <Adafruit_GFX.h>// melhor personalização do display
#include <esp_task_wdt.h> // se o sistema acabar travando, é reiniciado
#include <esp_system.h> // mostra pq reiniciou
#include "driver/gpio.h" // controle de baixo nivel dos pinos
#include <Adafruit_Sensor.h> //Biblioteca da Adafruit

// Pinos
#define pino_i2c_dado = 21; //pino de dados da com-i2c
#define pino_i2c_clock = 22; //pino clock da i2c
#define pino_sct = 34; //pino do sensor de corrente
#define pino_bot = 13; //botão de emergência
#define pino_rele_led = 17; //pino pra acionar o led de alerta via relé
#define pino_rele_mot = 16; //pino pra acionar o motor de desligamento via relé

// ADXL
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(); // Adxl objeto

void setup() {
 //parte do adxl
  Serial.begin(9600);
  if(!accel.begin()){ //se adxl não encontrado = erro
    Serial.println("ADXL não encontrado");
    while(1);
  }
}

void ADXL(){

sensors_event_t event; //detecta o movimento do ADXL
accel.getEvent(&event);

float x = event.acceleration.x;
float y = event.acceleration.y;
float z = event.acceleration.z;
float total = sqrt(x*x + y*y + z*z);

  Serial.print("X: ");
  Serial.print(x);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print("Total: ");
  Serial.print(total);
  Serial.print("  ");
  Serial.println("m/s^2");
}

void loop(){

  ADXL();
  delay(500);

}