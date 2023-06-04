#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "printf.h"
#include "nrf_to_nrf.h"

#define SENSEx D7
#define PULSEx D3
#define RELEXx D5
#define FV_EN_P D4
#define SAFETY1 D2

const unsigned long duration = 60000;
unsigned long start_timer;
unsigned long start_timerA;
unsigned long end_timerA;

int pulseWidth = 500;

float coef_1 = 16.9;
float coef_2 = 17.75;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SENSEx, INPUT);
  pinMode(PULSEx, OUTPUT);
  pinMode(RELEXx, OUTPUT);
  pinMode(SAFETY1, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(PIN_SPI_SS, OUTPUT); // slave select pin
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, OUTPUT);
  SPI.begin();
  
  pinMode(FV_EN_P, OUTPUT);
  digitalWrite(PIN_SPI_SS, HIGH); //deselect slave
  digitalWrite(FV_EN_P, HIGH);
  digitalWrite(SAFETY1, HIGH);
  digitalWrite(LEDR, HIGH);

  attachInterrupt(digitalPinToInterrupt(SENSEx), ISR, RISING);
  
  digitalWrite(PULSEx, LOW);
  digitalWrite(RELEXx, LOW);
  start_timer = millis();
}

void loop() {
  digitalWrite(LEDR, HIGH);
  start_timerA = millis();
  digitalWrite(RELEXx, HIGH);
  delayMicroseconds(350);
  digitalWrite(PULSEx, HIGH);

  writeSPI_Arduino(1, 10);
  delayMicroseconds(pulseWidth);
  digitalWrite(PULSEx, LOW);
  delayMicroseconds(pulseWidth*4);
  digitalWrite(RELEXx, LOW);
  end_timerA = millis();

  Serial.print("Execution time is: ");
  Serial.println(end_timerA - start_timerA);
  delay(20 - (end_timerA - start_timerA)); 

  if ((millis() - start_timer) >= duration) {
    digitalWrite(PIN_SPI_SS, HIGH); //deselect slave
    digitalWrite(FV_EN_P, LOW);
    digitalWrite(SAFETY1, LOW);
  }

}

void ISR() {
  digitalWrite(LEDR, LOW);
}

void writeSPI_Arduino(uint8_t channel, uint16_t pulse_current_mA) {
  uint16_t current_out;
  if (pulse_current_mA > 0 && pulse_current_mA <= 50) {
    current_out = pulse_current_mA * coef_1;
  } else if (pulse_current_mA > 50 && pulse_current_mA <= 170) {
    current_out = pulse_current_mA * coef_2;
  }
  uint8_t spiTxBuf[3];
  uint16_t temp16 = current_out << 4;
  uint8_t temp8 = channel;
  spiTxBuf[0] = 0x08;
  spiTxBuf[1] = (temp16 & 0xFF00) >> 8;
  spiTxBuf[2] = temp16 & 0x00FF;

  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(spiTxBuf, 3);
  digitalWrite(PIN_SPI_SS, HIGH);
}
