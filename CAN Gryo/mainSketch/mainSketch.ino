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
uint8_t messageOne[size] =  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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
  uint16_t pitch16 = sEul.pitch;
  uint16_t roll16 = sEul.roll;
  uint16_t head16 = sEul.head;

  uint8_t pitchOne = 0;
  uint8_t pitchTwo = 0;
  uint8_t rollOne = 0;
  uint8_t rollTwo = 0;
  uint8_t headOne = 0;
  uint8_t headTwo = 0;

  EncodeFloat(sEul.pitch, pitchOne, pitchTwo);
  EncodeFloat(sEul.roll, rollOne, rollTwo);
  EncodeFloat(sEul.head, headOne, headTwo);
  
  Serial.println("Pitch: " + String(sEul.pitch, 0) + ". Roll: " + String(sEul.roll, 0) + ". Head: " + String(sEul.head, 0));
  Serial.println("Pitch: " + String(pitchOne + pitchTwo) + ". Roll: " + String(rollOne + rollTwo) + ". Head: " + String(headOne + headTwo));
  Serial.println("");

  messageOne[0] = headOne;
  messageOne[1] = headTwo;
  messageOne[2] = pitchOne;
  messageOne[3] = pitchTwo;
  messageOne[4] = rollOne;
  messageOne[5] = rollTwo;

  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    ReadMessages();
  }
  else{
    SendCanMessage(messageOne);
  }

  // if (CAN.checkError() == CAN_CTRLERROR)
  // {
  //   Serial.println("ERROR");
  // }

  delay(100);
}

void EncodeFloat(float value, uint8_t &outOne, uint8_t &outTwo){
  const int byteValue = 255;
  const int maxValue = 360;
  int roundedValue = value;

  if (roundedValue < 0)
  {
    roundedValue = maxValue + roundedValue;
  }

  if (roundedValue <= byteValue)
  {
    outOne = roundedValue;
    outTwo = 0;
  }
  else
  {
    outOne = byteValue;
    outTwo = roundedValue - byteValue;
  }

}

void SendCanMessage(uint8_t message[]){
  Serial.println("Sending CAN");
  CAN.sendMsgBuf(canID, 1, size, message);

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