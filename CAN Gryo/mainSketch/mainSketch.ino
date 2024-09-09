#include <DFRobot_MCP2515.h>
#include "DFRobot_BNO055.h"
#include "Wire.h"
#include <SPI.h>

typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********
BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address

const int SPI_CS_PIN = 9;
DFRobot_MCP2515 CAN(SPI_CS_PIN);

char startChar = '$';
char endChar = '#';
char seperatorChar = ' ';
uint32_t canID = 0x00FFFF0C;

const int size = 10;
unsigned char msgOne[size] =  {0x24, 0x00, 0x00, 0xFF, 0xFF, 0xAA, 0xBB, 0xCC};



void setup()
{
  Serial.println("Board started.");

  Serial.begin(57600);
  bno.reset();

  // Wait until the gyro is connected.
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("Gryo failed.");
    //PrintMessage("0");
    delay(2000);
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

  Serial.println("CAN shield inited.");
  Serial.println("Board initialised. Loop starting.");
}

void loop()
{
  //BNO::sEulAnalog_t   sEul;
  //sEul = bno.getEul();
  //PrintMessage(seperatorChar + "1" + seperatorChar + String(sEul.pitch, 3) + seperatorChar + String(sEul.roll, 3) + seperatorChar + String(sEul.head, 3));

  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    ReadMessages();
  }
  else{
    PrintMessage("");
  }

  if (CAN.checkError() == CAN_CTRLERROR)
  {
    Serial.println("ERROR");
  }

  delay(100);
}

void PrintMessage(String message)
{
    SendCanMessage(msgOne, "1");
}

void SendCanMessage(unsigned char message[], String name){
  // FFFF07 - 16776967
  Serial.print("Snd msg: ");
  for (int i = 0; i < size; i++)
  {
    Serial.print(message[i]);
    Serial.print(", ");
  }
  Serial.println("");

  if (MCP2515_OK == CAN.sendMsgBuf(0x00FFFF0C, 1, size, message)){
    Serial.println("Sent CAN " + name);
  }
  else{
    Serial.println("Failed to send CAN " + name);
  }
}

void ReadMessages(){
  // if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  // {
    unsigned char len = 0;
    unsigned char buf[32];

    uint32_t id = CAN.getCanId();
    Serial.print(id);
    Serial.print("\t\t");

    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    for(int i = 0; i < len; i++)    // print the data
    {
        Serial.print(buf[i]);
        Serial.print("\t");
    }
    Serial.println();
  // }
}

void PrintAvailableCanPorts(){
  for (int i = 0; i < 100; i++)
  {
    Serial.println("Trying " + String(i));
    DFRobot_MCP2515 CANX(i);
    if(CAN_OK != CANX.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
    {
        Serial.println("DFROBOT's CAN BUS Shield init fail");
        delay(100);
    }
    else{
        Serial.println("CAN-Bus shield inited");
    }
  }
}


