#include <DFRobot_MCP2515.h>
#include "Wire.h"
#include <SPI.h>
#include <PID_v1.h>

// CAN-BUS
const int SPI_CS_PIN = 9;
DFRobot_MCP2515 CAN(SPI_CS_PIN);
const int arduinoAddress = 9;
int canErrorCount = 0;

// Message Out
const int size = 8;
uint8_t messageOne[size] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned long lastMessageSent = 0;
const unsigned long delayBetweenSendingMessagesMs = 2000;

// Station keeping
bool stationKeepingEnabled = false;
bool newPosition = false;
double holdLatitude;
double holdLongitude;
uint8_t yawMsb;
uint8_t yawLsb;
uint8_t foreMsb;
uint8_t foreLsb;
uint8_t latMsb;
uint8_t latLsb;

// Data from RovIns
double currentLatitude;
double currentLongitude;
double currentDepth;

// PID
//Define Variables we'll be connecting to
double latitudeOutput, longitudeOutput;
double latitudeKp = 2, latitudeKi = 5, latitudeKd = 1;
double longitudeKp = 2, longitudeKi = 5, longitudeKd = 1;
PID latitudePID(&currentLatitude, &latitudeOutput, &holdLatitude, latitudeKp, latitudeKi, latitudeKd, DIRECT);
PID longitudePID(&currentLongitude, &longitudeOutput, &holdLongitude, longitudeKp, longitudeKi, longitudeKd, DIRECT);

// Debug
bool messageSent = false;

void setup() {
  Serial.begin(57600);
  InitialiseCanShield();
}

void InitialiseCanShield() {
  //Serial.flush();
  //Serial.println("\nInitialising CAN shield...");

  bool canInit = false;
  do {
    if (CAN_OK != CAN.begin(CAN_500KBPS)) {
      //Serial.println("CAN shield failed to init.");
      delay(100);
    } else {
      canInit = true;
    }
  } while (!canInit);

  //Serial.println("CAN shield initialised.");
}

void loop() {
  messageSent = false;

  if (stationKeepingEnabled) {
    RovInsReadMessage();
  }

  if (CAN.checkError() != CAN_CTRLERROR) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      ReadCanBusMessage();
    }

    if (stationKeepingEnabled) {
      HandleStationKeeping();
    }
  } else {
    Serial.println("CAN error. " + String(canErrorCount));
    canErrorCount++;

    if (canErrorCount > 20) {
      canErrorCount = 0;
      InitialiseCanShield();
    }
  }

  unsigned long now = millis();
  if ((now - delayBetweenSendingMessagesMs) > lastMessageSent)
  {
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
    //Serial.println("Heartbeat: " + String(stationKeepingEnabled));
  }

  String message = "";

  if (messageSent) {
    message += "Msg Sent. ";
  }

  if (stationKeepingEnabled) {
    message += "Sk.";
  }

  if (message != "") {
    message += "\n";
  }
  //Serial.print(message + "Lat c" + String(currentLatitude) + ", o" + String(latitudeOutput) + ", h" + String(holdLatitude) + "\nLng c" + String(currentLongitude) + ", o" + String(longitudeOutput) + ", h" + String(holdLatitude) + "\n");
}

void ReadCanBusMessage() {
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
            SaveCurrentPositionForStationKeeping();
            
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

void HandleStationKeeping() {
  //unsigned long now = millis();
  //if ((now - delayBetweenSendingMessagesMs) > lastMessageSent && newPosition)
  //{
  //lastMessageSent = now;

  latitudePID.Compute();
  longitudePID.Compute();

  // Send thruster message
  SendThrusterCommand(latitudeOutput * 50, longitudeOutput * 50);
  messageSent = true;

  // When a message is send, clear the error count.
  canErrorCount = 0;
  //}
}

void SaveCurrentPositionForStationKeeping() {
  holdLatitude = currentLatitude;
  holdLongitude = currentLongitude;
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
    String message = Serial.readStringUntil('\r');

    // The APOS_PSIMLBP message is 93 characters long
    if (message.length() > 2) {
      if (message[0] == '$' && message[1] == 'P') {
        String messages[13];
        String data = "";
        int index = 0;

        // Split the message into individual data packets that can only contain numbers, letters, '.' and '*'.
        for (int i = 0; i < message.length(); i++) {
          if (message[i] != ',') {
            if (isAlphaNumeric(message[i]) || message[i] == '.' || message[i] == '*') {
              data += message[i];
            }
          } else {
            messages[index++] = data;
            data = "";
          }
        }

        messages[index++] = data;

        String header = messages[0];
        if (header.length() == 7 && header == "PSIMLBP")  // header[0] == 'P' && header[1] == 'S' && header[2] == 'I' && header[3] == 'M' && header[4] == 'L' && header[5] == 'B' && header[6] == 'P')
        {
          //Serial.println("RovIns -- Message read");

          // For debugging.
          // String dataStr = "";
          // if (index > 0)
          // {
          //   for (int i = 0; i < index; i++)
          //   {
          //     dataStr += String(i) + ": " + messages[i] + "\n";
          //   }
          // }
          // Serial.println("\n--Valid data--\n" + String(message.length()) + " - " + dataStr);

          // [6] = lat, [7] = long, [8] = depth
          currentLatitude = messages[6].toFloat();
          currentLongitude = messages[7].toFloat();
          currentDepth = messages[8].toFloat();
          newPosition = true;

          //Serial.println("RovIns: lat: " + String(currentLatitude) + ", long: " + String(currentLongitude));
        } else {
          //Serial.println("--BAD DATA--");
        }
      }
    }
  }
}
