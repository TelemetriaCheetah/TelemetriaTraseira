#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Adafruit_GPS.h>
#include "Cheetah.h"

#define CAN_RX_BUFFER_SIZE 81
#define GPS_RX_BUFFER_SIZE 4
#define GPSSerial Serial1
#define GPSECHO false
#define TENSAO_GLV 35
#define HALL_GLV 34

static uint8_t taskCoreZero = 0;
static uint8_t taskCoreOne  = 1;
uint32_t timer = millis();

CheetahSerial serial;
CheetahCAN can(26);
Adafruit_GPS GPS(&GPSSerial);
Acelerometro accel;

uint16_t canRxBuffer[CAN_RX_BUFFER_SIZE];
uint32_t gpsRxBuffer[GPS_RX_BUFFER_SIZE];

void readCanBuffer(void *pvParameters);
void readSensor(void *pvParameters);
void sendData(void *pvParameters);
void readGPS();

#endif