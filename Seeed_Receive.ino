#include <SPI.h>
//const int SPI_CS_PIN = 9;  //untuk module MCP2515
const int SPI_CS_PIN = 10;  //untuk Arduino Shield
const int CAN_INT_PIN = 2;
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

void setup() {
  // put your setup code here, to run once:
    SERIAL_PORT_MONITOR.begin(115200);

    while (CAN_OK != CAN.begin(CAN_250KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
}
uint32_t id;
uint8_t  len;
byte data[8] = {0};

void loop() {
  // put your main code here, to run repeatedly:
    if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data coming
        CAN.readMsgBuf(&len, data);    // read data,  len: data length, buf: data buf

        id = CAN.getCanId();

        SERIAL_PORT_MONITOR.println("-----------------------------");
        SERIAL_PORT_MONITOR.print("Get data from ID: 0x");
        SERIAL_PORT_MONITOR.println(id, HEX);

        for (int i = 0; i < len; i++) { // print the data
            SERIAL_PORT_MONITOR.print(data[i], BIN);
            SERIAL_PORT_MONITOR.print("\t");
        }
        SERIAL_PORT_MONITOR.println();
    }
}
