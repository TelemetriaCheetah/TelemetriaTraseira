#include "main.h"
#define DEBUG 0
void setup() 
{
  Serial.begin(115200);
  //delay(5000);
  can.beginCAN();
  if(!DEBUG)
    xTaskCreatePinnedToCore(readCanBuffer,"readCanBuffer",10000,NULL,2,NULL,taskCoreZero);
  //xTaskCreatePinnedToCore(readSensor,"readSensor",10000,NULL,1,NULL,taskCoreOne);
  if(!DEBUG)
    xTaskCreatePinnedToCore(sendData,"sendData",10000,NULL,2,NULL,taskCoreOne);
  //GPS.begin(9600);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);
  //delay(1000);
  //Serial.println("iniciando");
}

void loop() 
{
  //Serial.print("CANRX 0: ");
  //Serial.println(canRxBuffer[0]);
  //delay(100);
  if(DEBUG)
  {
    can.testeCan();
  }
}

void readSensor(void *pvParameters)
{
  while (true)
  { 
    // readGPS();
    accel.leituraVariaveis();
    delay(10);
  }
}

void sendData(void *pvParameters)
{
  while (true)
  {
    // Serial.println("Teste");
    serial.addAnalogSensor(canRxBuffer[1]); //TPS1 
    serial.addAnalogSensor(canRxBuffer[2]); //TPS2 
    serial.addAnalogSensor(canRxBuffer[4]); //Rotação FD 
    serial.addAnalogSensor(canRxBuffer[5]); //Rotação FE 
    serial.addAnalogSensor(canRxBuffer[6]); //Rotação TD 
    serial.addAnalogSensor(canRxBuffer[7]); //Rotação TE 
    serial.addAnalogSensor(canRxBuffer[8]); //Temperatura centro disco FD
    serial.addAnalogSensor(canRxBuffer[9]); //Temperatura borda disco FD
    serial.addAnalogSensor(canRxBuffer[10]);//Temperatura centro disco FE
    serial.addAnalogSensor(canRxBuffer[11]);//Temperatura borda disco FE
    serial.addAnalogSensor(canRxBuffer[12]);//Temperatura centro disco TD
    serial.addAnalogSensor(canRxBuffer[13]);//Temperatura borda disco TD
    serial.addAnalogSensor(canRxBuffer[14]);//Temperatura centro disco TE
    serial.addAnalogSensor(canRxBuffer[15]);//Temperatura borda disco TE
    serial.addAnalogSensor(canRxBuffer[16]);//Encoder do volante
    serial.addAnalogSensor(gpsRxBuffer[0]); //Latitude GNSS
    serial.addAnalogSensor(gpsRxBuffer[0] >> 15); //Latitude GNSS
    serial.addAnalogSensor(gpsRxBuffer[1]);       //Longitude GNSS
    serial.addAnalogSensor(gpsRxBuffer[1] >> 15); //Longitude GNSS
    serial.addAnalogSensor(gpsRxBuffer[2]);       //Sat count
    serial.addAnalogSensor(gpsRxBuffer[3]);       //HDOP
    // serial.addAnalogSensor(123);
    // serial.addAnalogSensor(696);
    // serial.addAnalogSensor(4442);
    // serial.addAnalogSensor(350);
    // serial.addAnalogSensor(420);
    // serial.addAnalogSensor(1337);
    // serial.addAnalogSensor(0);
    // serial.addAnalogSensor(0);
    // serial.addAnalogSensor(0);
    // serial.addAnalogSensor(0);
    serial.addAnalogSensor(accel.accelX()); //Aceleração eixo X
    serial.addAnalogSensor(accel.accelY()); //Aceleração eixo Y
    serial.addAnalogSensor(accel.accelZ()); //Aceleração eixo Z
    serial.addAnalogSensor(accel.gyroX());  //Giroscópio eixo X
    serial.addAnalogSensor(accel.gyroY());  //Giroscópio eixo Y
    serial.addAnalogSensor(accel.gyroZ());  //Giroscópio eixo Z
    serial.addAnalogSensor(accel.magX());   //Magnetômetro eixo X
    serial.addAnalogSensor(accel.magY());   //Magnetômetro eixo Y
    serial.addAnalogSensor(accel.magZ());   //Magnetômetro eixo Z
    serial.addAnalogSensor(accel.temp());   //Temperatura
    
    for(int i = 31 ; i <= 80 ; i++)
    {
      if(i == 55)
      {
        serial.addAnalogSensor(analogRead(TENSAO_GLV));
      }
      else if(i == 70)
      {
        serial.addAnalogSensor(analogRead(HALL_GLV));
      }
      else
      {
        serial.addAnalogSensor(canRxBuffer[i]);
      }
    }
    serial.addAnalogSensor(65342);
    serial.sendPayload();
    delay(100);
  }
}

void readCanBuffer(void *pvParameters)
{
  while (true)
  {
      if(can.readMessage())
      {
        uint16_t id = can.getMsgId();
        uint8_t * msg = can.getMsg();
        switch (id)
        {
          case 0x00:                                  // Telemetria Frontal
            canRxBuffer[ 1] = msg[0] | msg[1] << 8;   // SA1
            canRxBuffer[ 2] = msg[2] | msg[3] << 8;   // SA2
            canRxBuffer[76] = msg[4] | msg[5] << 8;   // SA76
            canRxBuffer[80] = msg[6] | msg[7] << 8;   // Flags Inversor
            break;
          case 0x01:                                  // Roda Frontal Esquerda
            canRxBuffer[10] = msg[0] | msg[1] << 8;   // SA10
            canRxBuffer[11] = msg[2] | msg[3] << 8;   // SA11
            canRxBuffer[ 5] = msg[4] | msg[5] << 8;   // SA5
            break;
          case 0x02:                                  // Roda Frontal Direita
            canRxBuffer[ 8] = msg[0] | msg[1] << 8;   // SA8
            canRxBuffer[ 9] = msg[2] | msg[3] << 8;   // SA9
            canRxBuffer[ 4] = msg[4] | msg[5] << 8;   // SA4
            break;
          case 0x03:                                  // Roda Traseira Esquerda
            canRxBuffer[14] = msg[0] | msg[1] << 8;   // SA14
            canRxBuffer[15] = msg[2] | msg[3] << 8;   // SA15
            canRxBuffer[ 7] = msg[4] | msg[5] << 8;   // SA7
            break;
          case 0x04:                                  // Roda Traseira Direita
            canRxBuffer[12] = msg[0] | msg[1] << 8;   // SA12
            canRxBuffer[13] = msg[2] | msg[3] << 8;   // SA13
            canRxBuffer[ 6] = msg[4] | msg[5] << 8;   // SA6
            break;
          case 0x05:                                  // Telemetria Frontal
            canRxBuffer[ 0] = msg[0] | msg[1] << 8;   // SD
            canRxBuffer[16] = msg[2] | msg[3] << 8;   // SA16
            canRxBuffer[78] = msg[4] | msg[5] << 8;   // SA78
            canRxBuffer[79] = msg[6] | msg[7] << 8;   // SA79
            break;
          case 0x06:                                  // Circuito de segurança
            canRxBuffer[ 0] = msg[0] | msg[1] << 8;   // SD
            canRxBuffer[71] = msg[2] | msg[3] << 8;   // SA71
            canRxBuffer[72] = msg[4] | msg[5] << 8;   // SA72
            canRxBuffer[73] = msg[6] | msg[7] << 8;   // SA73
            break;
          case 0x07:                                  // Circuito de segurança
            canRxBuffer[74] = msg[0] | msg[1] << 8;   // SA74
            canRxBuffer[75] = msg[2] | msg[3] << 8;   // SA75
            canRxBuffer[77] = msg[4] | msg[5] << 8;   // SA77
            break;
          case 0x08:                                  // BMS
            canRxBuffer[31] = msg[0] | msg[1] << 8;   // SA31
            canRxBuffer[32] = msg[2] | msg[3] << 8;   // SA32
            canRxBuffer[33] = msg[4] | msg[5] << 8;   // SA33
            canRxBuffer[34] = msg[6] | msg[7] << 8;   // SA34
            break;
          case 0x09:                                  // BMS
            canRxBuffer[35] = msg[0] | msg[1] << 8;   // SA35
            canRxBuffer[36] = msg[2] | msg[3] << 8;   // SA36
            canRxBuffer[37] = msg[4] | msg[5] << 8;   // SA37
            canRxBuffer[38] = msg[6] | msg[7] << 8;   // SA38
            break;
          case 0x10:                                  // BMS
            canRxBuffer[39] = msg[0] | msg[1] << 8;   // SA39
            canRxBuffer[40] = msg[2] | msg[3] << 8;   // SA40
            canRxBuffer[41] = msg[4] | msg[5] << 8;   // SA41
            canRxBuffer[42] = msg[6] | msg[7] << 8;   // SA42
            break;
          case 0x11:                                  // BMS
            canRxBuffer[43] = msg[0] | msg[1] << 8;   // SA43
            canRxBuffer[44] = msg[2] | msg[3] << 8;   // SA44
            canRxBuffer[45] = msg[4] | msg[5] << 8;   // SA45
            canRxBuffer[46] = msg[6] | msg[7] << 8;   // SA46
            break;
          case 0x12:                                  // BMS
            canRxBuffer[47] = msg[0] | msg[1] << 8;   // SA47
            canRxBuffer[48] = msg[2] | msg[3] << 8;   // SA48
            canRxBuffer[49] = msg[4] | msg[5] << 8;   // SA49
            canRxBuffer[50] = msg[6] | msg[7] << 8;   // SA50
            break;
          case 0x013:                                 // BMS
            canRxBuffer[51] = msg[0] | msg[1] << 8;   // SA51
            canRxBuffer[52] = msg[2] | msg[3] << 8;   // SA52
            canRxBuffer[53] = msg[4] | msg[5] << 8;   // SA53
            canRxBuffer[54] = msg[6] | msg[7] << 8;   // SA54
            break;
          case 0x014:                                 // Inversor
            canRxBuffer[56] = msg[0] | msg[1] << 8;   // SA56
            canRxBuffer[57] = msg[2] | msg[3] << 8;   // SA57
            canRxBuffer[58] = msg[4] | msg[5] << 8;   // SA58
            canRxBuffer[59] = msg[6] | msg[7] << 8;   // SA59
            break;
          case 0x015:                                 // Inversor
            canRxBuffer[60] = msg[0] | msg[1] << 8;   // SA60
            canRxBuffer[61] = msg[2] | msg[3] << 8;   // SA61
            canRxBuffer[62] = msg[4] | msg[5] << 8;   // SA62
            canRxBuffer[63] = msg[6] | msg[7] << 8;   // SA63
            break;
          case 0x016:                                 // Inversor
            canRxBuffer[64] = msg[0] | msg[1] << 8;   // SA64
            canRxBuffer[65] = msg[2] | msg[3] << 8;   // SA65
            canRxBuffer[66] = msg[4] | msg[5] << 8;   // SA66
            canRxBuffer[67] = msg[6] | msg[7] << 8;   // SA67
            break;
          case 0x017:                                 // Inversor
            canRxBuffer[68] = msg[0] | msg[1] << 8;   // SA68
            canRxBuffer[69] = msg[2] | msg[3] << 8;   // SA69
            break;
          default:
            break;
        }
      }
    delay(10);
  }
}

// void readGPS()
// {
//   char c = GPS.read();
//   if (GPS.newNMEAreceived()) 
//   {
//     if (!GPS.parse(GPS.lastNMEA()))
//       return;
//   }
//   if (millis() - timer > 2000)
//   {
//     gpsRxBuffer[0] = GPS.latitude;
//     gpsRxBuffer[1] = GPS.longitude;
//     gpsRxBuffer[2] = GPS.satellites;
//     gpsRxBuffer[3] = GPS.fixquality;
//   }
// }