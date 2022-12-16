#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "mcp2515_can.h"
#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int SPI_CS_PIN = PB6;    //PB6 untuk STM32
const int CAN_INT_PIN = 2;    //10 untuk Arduino
int state = 0;
byte x = 0b00000000;
byte y = 0b00000000;
byte z = 0b00000000;

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    sensors.begin();
    while (CAN_OK != CAN.begin(CAN_250KBPS)) {             // init can bus : baudrate = 500k
        Serial.println("CAN init fail, retry...");        // Baudrate Modul = 500k/250k, MCP_8hz
        delay(100);                                       // Baudrate Shield = 500k/250k
    }
    Serial.println("CAN init ok!");
}

void loop() {
uint32_t id = 0x0CF00400;
uint8_t  len;
int rdData = random(0,150);
state ++;
sensors.requestTemperatures();
int sendSens = sensors.getTempCByIndex(0);
switch (state%2 == 0){      //apabila dibagi 2 habis
  case 0:                   //apabila nilainya 0
  bitWrite(x, 0, 1);        //masukan nilai 1 pada bit 0, x merupakan variable tampung
  break;
  case 1:
  bitWrite(x, 0, 0);
  break;
  
  }
switch (state%3 == 0){
  case 0:
  bitWrite(x, 1, 1);
  break;
  case 1:
  bitWrite(x, 1, 0);
  break;
  
  }
switch (state%5 == 0){
  case 0:
  bitWrite(x, 2, 1);
  break;
  case 1:
  bitWrite(x, 3, 0);
  break;
  
  }
    
byte data[8] = {0, 0, 0, rdData, z, y, x, sendSens};
  // put your main code here, to run repeatedly:
Serial.print("Sending With ID: ");
Serial.println(id, HEX);
CAN.sendMsgBuf(id, 1, 0, 8, data);
for(int i=0; i<8; i++)
{
  Serial.print(data[i]);
  Serial.print("\t");
  }
  Serial.println();
}
