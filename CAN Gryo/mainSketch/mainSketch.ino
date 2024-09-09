#include <DFRobot_MCP2515.h>
#include "DFRobot_BNO055.h"
#include "Wire.h"
#include <SPI.h>

// Gyro
typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********
BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address

//CAN-BUS
const int SPI_CS_PIN = 9;
DFRobot_MCP2515 CAN(SPI_CS_PIN);

uint32_t canID = 0x00FFFF0C;
const int size = 8;
unsigned char msgOne[size] =  {0x24, 0x00, 0x00, 0xFF, 0xFF, 0xAA, 0xBB, 0xDD};

void setup()
{
  //Serial.println("Board started.");

  Serial.begin(57600);
  bno.reset();

  while(bno.begin() != BNO::eStatusOK) {
    //Serial.println("Gryo failed.");
    delay(500);
  }

  Serial.println("Gryo inited.");

  bool canInit = false;
  do {
      //CAN.init();   //must initialize the Can interface here!
      if(CAN_OK != CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
      {
          Serial.println("CAN shield failed to init.");
          delay(100);
      }
      else{
        canInit = true;
      }
  }while(!canInit);

  //Serial.println("CAN shield inited. Board initialised. Loop starting.");
}

void loop()
{
  BNO::sEulAnalog_t   sEul;
  sEul = bno.getEul();
  //PrintMessage(seperatorChar + "1" + seperatorChar + String(sEul.pitch, 3) + seperatorChar + String(sEul.roll, 3) + seperatorChar + String(sEul.head, 3));

  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    ReadMessages();
  }
  else{
    SendCanMessage(msgOne);
  }

  // if (CAN.checkError() == CAN_CTRLERROR)
  // {
  //   Serial.println("ERROR");
  // }

  delay(100);
}

void SendCanMessage(unsigned char message[]){
  Serial.println("Sending CAN");
  CAN.sendMsgBuf(0x00FFFF0C, 1, size, message);

  // if (MCP2515_OK == CAN.sendMsgBuf(0x00FFFF0C, 1, size, message)){
  //   Serial.println("Sent CAN ");
  // }
  // else{
  //   Serial.println("Failed to send CAN ");
  // }
}

void ReadMessages(){
    unsigned char len = 0;
    unsigned char buf[32];

    // uint32_t id = CAN.getCanId();
    // Serial.print(id);
    // Serial.print("\t\t");
    Serial.println("Recieving CAN");
    CAN.readMsgBuf(&len, buf);

    // for(int i = 0; i < len; i++)
    // {
    //     Serial.print(buf[i]);
    //     Serial.print("\t");
    // }
    // Serial.println();
}