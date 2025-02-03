#include "Wire.h"
#include <SPI.h>
#include <PID_v1.h>

// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13
#include <SPI.h>

#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif     

// CAN-BUS
const int arduinoAddress = 9;
int canErrorCount = 0;

// Message Out
const int size = 8;
uint8_t messageOne[size] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned long lastMessageSent = 0;
const unsigned long delayBetweenSendingMessagesMs = 500;

// Station keeping
bool stationKeepingEnabled = false;
bool newPosition = false;
double holdX;
double holdY;
uint8_t yawMsb;
uint8_t yawLsb;
uint8_t foreMsb;
uint8_t foreLsb;
uint8_t latMsb;
uint8_t latLsb;

// Data from RovIns
double currentXAccumulation;
double currentYAccumulation;
double currentDepth;
bool newVelocityData = false;

// PID
//Define Variables we'll be connecting to
double xOutput, yOutput;
double xKp = 2, xKi = 5, xKd = 1;
double yKp = 2, yKi = 5, yKd = 1;
PID latitudePID(&currentXAccumulation, &xOutput, &holdX, xKp, xKi, xKd, DIRECT);
PID longitudePID(&currentYAccumulation, &yOutput, &holdY, yKp, yKi, yKd, DIRECT);

// Debug
bool messageSent = false;

void setup() {
  Serial.begin(57600);
  InitialiseCanShield();
}

void InitialiseCanShield() {
  SERIAL_PORT_MONITOR.begin(57600);

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
}

void loop() {
  messageSent = false;

  
  if (CAN.checkError() != CAN_CTRLERROR) {
    ReadCanBusMessage();
  } else {
    Serial.println("E can " + String(canErrorCount));
    canErrorCount++;

    if (canErrorCount > 30) {
      canErrorCount = 0;
      InitialiseCanShield();
    }
  }

  unsigned long now = millis();
  if ((now - delayBetweenSendingMessagesMs) > lastMessageSent)
  {
    if (stationKeepingEnabled) {
      RovInsReadMessage();
      //HandleStationKeeping();
    }

    lastMessageSent = now;

    messageOne[0] = 0;
    messageOne[1] = 0;
    messageOne[2] = 0;
    messageOne[3] = 0;
    messageOne[4] = 0;
    messageOne[5] = stationKeepingEnabled ? 2 : 1;
    messageOne[6] = 0;
    messageOne[7] = 0;

    SendCanMessage(0xFF8F, 9, messageOne);
    Serial.println("Heartbeat: " + String(stationKeepingEnabled));
  }
}

void ReadCanBusMessage() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    uint8_t len = 0;
    uint8_t buf[16];

    CAN.readMsgBuf(&len, buf);

    uint32_t id = CAN.getCanId();
    uint32_t command = id >> 8;
    uint32_t address = id - (command << 8);

    // DEBUG: Print out the message split into it's parts.
    // String commandStr = String(command, HEX);
    // commandStr.toUpperCase();
    // String dataStr = "";
    // for (int i = 0; i < len; i++) {
    //   dataStr += String(buf[i], HEX) + ", ";
    // }
    // Serial.println("Message from address: " + String(address) + ", command: " + commandStr + ". DLC: " + String(len) + ". Data: " + dataStr);

    if (address == arduinoAddress) {

      //Serial.println("CanBus -- Message read " + String(command));

      // System Mode Control command
      if (command == 0xFFAA) {
        if (buf[5] == 1 || buf[5] == 2) {
          bool newStationKeeping = buf[5] == 2;

          // If value has been toggled.
          if (newStationKeeping != stationKeepingEnabled) {
            stationKeepingEnabled = newStationKeeping;

            if (stationKeepingEnabled) {
              holdX = 0;
              holdY = 0;
              
              latitudePID.SetMode(AUTOMATIC);
              longitudePID.SetMode(AUTOMATIC);

              Serial.println("Station keeping enabled.");
              newPosition = false;
            } else {
              Serial.println("Station keeping disabled.");
            }
          }
        }
      }
    }
  }
}

void HandleStationKeeping() {
  //unsigned long now = millis();
  //if ((now - delayBetweenSendingMessagesMs) > lastMessageSent && newPosition)
  //{
  //lastMessageSent = now;

  if (newVelocityData)
  {
    newVelocityData = false;
    latitudePID.Compute();
    longitudePID.Compute();

    //Serial.println("PID processed: X in " + String(currentXAccumulation) + ", x out " + String(xOutput) + ".");

    
    //Serial.availableForWrite()
    // Send thruster message
    //SendThrusterCommand(xOutput, yOutput); // READD
    messageSent = true;

    // When a message is send, clear the error count.
    //canErrorCount = 0;
  }
  //}
}

void SaveCurrentPositionForStationKeeping() {
  holdX = 0;
  holdY = 0;
}

void SendCanMessage(uint32_t id, uint32_t address, uint8_t message[]) {
  uint32_t fullId = (id << 8) + address;
  CAN.sendMsgBuf(fullId, 1, size, message);
}

void SendThrusterCommand(float foreAft, float lateral) {
  messageOne[0] = (byte)foreAft >> 8;    // Fore Msb
  messageOne[1] = (byte)foreAft & 0xFF;  // Fore Lsb
  messageOne[2] = 0;                     // Not used (vertical)
  messageOne[3] = 0;                     // Not used (vertical)
  messageOne[4] = yawMsb;
  messageOne[5] = yawLsb;
  messageOne[6] = (byte)lateral >> 8;    // Lateral Msb
  messageOne[7] = (byte)lateral & 0xFF;  // Lateral Lsb

  //Serial.println("Sent thruster command: F/A: " + String(foreAft) + ". Lateral: " + String(lateral) + ". Yaw: " + String(yawMsb) + " " + String(yawLsb));
  //Serial.println("Sent CMD: " + String(foreAft) + ", " + String(lateral) + ". Lat: " + String(currentLatitude) + " - " + String(holdLatitude));

  SendCanMessage(0xFFFC, 0, messageOne);
}

void RovInsReadMessage() {
  if (Serial.available()) {

    bool foundVelocityMessage = false;
    const int maxAttempts = 7;
    int counter = 0;

    char buffer[10];
    String message;
    do
    {
      //message = Serial.readBytes(buffer, 5);
      message = Serial.readStringUntil(':');
      // if (message.length() > 24)
      // {
      //   if (message.charAt(0) == 'B' && message.charAt(1) == 'I' && message.charAt(2) == ',')
      //   {
      //     foundVelocityMessage = true;
      //   }
      // }

      counter++;
    }
    while (Serial.available() && !foundVelocityMessage && counter < maxAttempts);
    
    // if (foundVelocityMessage)
    // {
    //   int xVelocity = message.substring(3, 9).toInt();
    //   int yVelocity = message.substring(10, 16).toInt();

    //   currentXAccumulation += xVelocity;
    //   currentYAccumulation += yVelocity;

    //   newVelocityData = true;

    //   //Serial.println("Data: " + String(xVelocity) + "x, " + String(yVelocity) + "y.");
    // }
  }
}