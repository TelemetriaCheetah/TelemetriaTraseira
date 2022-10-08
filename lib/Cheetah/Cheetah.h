#ifndef CHEETAH_H
#define CHEETAH_H
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define MSG_SIZE 165 //EM BYTES, CONSULTAR DOCUMENTACAO
#define N_SENSORES_MEDICAO 81
#define N_SENSORES_DISCRETO 8
#define CAN0_INT 27
#include "Arduino.h"
#include "mcpcan.h"
#include <Wire.h>
#include <HX711.h>
#include <config.h>

class CheetahSerial
{
  private:
    byte payload[MSG_SIZE];
    uint16_t digital[N_SENSORES_DISCRETO];
    uint16_t contA;
    uint16_t contD;
    bool subindo;
    unsigned int vel;
  public:
    CheetahSerial();
    void sendPayload();
    void addAnalogSensor(uint16_t value);
    void addDigitalSensor(uint16_t value);
};

class Acelerometro
{
  private:
   uint16_t variaveis[10];
   const int MPU=0x68;
  public:
   Acelerometro();
   void leituraVariaveis();
   uint16_t accelX();
   uint16_t accelY();
   uint16_t accelZ();
   uint16_t gyroX();
   uint16_t gyroY();
   uint16_t gyroZ();
   uint16_t magX();
   uint16_t magY();
   uint16_t magZ();
   uint16_t temp();
};

class CheetahCAN : public MCP_CAN
{
  using MCP_CAN::MCP_CAN;
  private:
    byte payload[8];
    uint8_t cont8;
    long unsigned int rxId;
    unsigned char rxLen = 0;
    unsigned char rxBuf[8];
    uint16_t init_status;
    unsigned char len = 0;
    char msgString[128];   
  public:
    void addToPayload8(byte value);
    void testeCan();
    uint8_t beginCAN();
    bool readMessage();
    uint8_t sendMessage(uint16_t id);
    uint16_t getMsgId();
    uint16_t getMsgLen();
    byte* getMsg();
};

// class CelulaDeCarga : public HX711
// {
//   public:
//     CelulaDeCarga();
//     uint16_t testeCelula();
//     uint16_t readCellValue();
// };

#endif
