#include <DFRobot_MCP2515.h> // Can Bus
#include "Wire.h"
#include <SPI.h>
#include <PID_v1.h>
#include <Ethernet.h>

// CAN-BUS
const int SPI_CS_PIN = 9;
DFRobot_MCP2515 CAN(SPI_CS_PIN);
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

// Ethernet
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x24, 0x16
};
IPAddress ip(192, 168, 137, 177);

// This port must match the port that tcp is being sent over.
int port = 10002;

// Enter the IP address of the server you're connecting to. This should match the comupters IPv4 address.
// If connecting to a Windows device, this is configured through:
// Contorl Panel > Network & Sharing Center > Ethernet 2 > Properties > TCP/IPv4 > IP address.
IPAddress server(192, 168, 137, 1);
int connectionFailedCounter = 0;
EthernetClient client;

// Debug
bool messageSent = false;

void setup() {
  Serial.begin(57600);
  while (!Serial) ;

  Serial.println("Serial initialisation - COMPLETE");

  InitialiseCanShield();
  InitialiseEthernetShield();
}

void InitialiseCanShield() {
  Serial.println("CAN Shield Initialisation - START");

  while (CAN_OK != CAN.begin(CAN_500KBPS)){
      Serial.println("CAN Shield Initialisation - ERROR");
      delay(100);
  }
  
  Serial.println("CAN Shield Initialisation - SUCCESS");
}

void InitialiseEthernetShield(){
  // Start the Ethernet connection.
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present.
  while (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet Shield Initialisation - ERROR - Ethernet shield was not found");
    delay(500);
  }

  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet Shield Initialisation - ERROR - cable is not connected.");
    delay(500);
  }

  // Give the Ethernet shield a second to initialize.
  delay(1000);

  Serial.println("Ethernet Shield Initialisation - COMPLETE.");
}

void loop() {
  HandleCanBus();
  HandleEthernetShieldConnection();
  //HandleStationKeeping();
}

void HandleCanBus(){
  if (CAN.checkError() != CAN_CTRLERROR) {
    ReadCanBusMessage();
  } else {
    Serial.println("Can bus RT - Error " + String(canErrorCount));
    canErrorCount++;

    if (canErrorCount > 30) {
      canErrorCount = 0;
      InitialiseCanShield();
    }
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

    PrintCanMessage(id, buf, len);

    if (address == arduinoAddress) {

      PrintCanMessage(id, buf, len);

      if (command == 0xFFAA) {
        HandleSystemModeControlCommand(buf);
      }
    }
  }
}

void HandleSystemModeControlCommand(uint8_t buf[]){
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

void PrintCanMessage(uint32_t id, uint8_t buf[], uint8_t len){
  uint32_t command = id >> 8;
  uint32_t address = id - (command << 8);

  String commandStr = String(command, HEX);
  commandStr.toUpperCase();
  String dataStr = "";
  for (int i = 0; i < len; i++) {
    dataStr += String(buf[i], HEX) + ", ";
  }

  Serial.println("CAN Message: Address: " + String(address) + ". Command: " + commandStr + ". DLC: " + String(len) + ". Data: " + dataStr);
}

void SendCanMessage(uint32_t id, uint32_t address, uint8_t message[]) {
  uint32_t fullId = (id << 8) + address;
  CAN.sendMsgBuf(fullId, 1, size, message);
}

void HandleEthernetShieldConnection(){
  if (Ethernet.hardwareStatus() != EthernetNoHardware && Ethernet.linkStatus() != LinkOFF && connectionFailedCounter < 20)
  {
    if (!client.connected()){

      // Disconnect from previous connection.
      client.stop();

      // if you get a connection, report back via serial:
      if (client.connect(server, 10002)) {
        Serial.println("Ethernet Shield RT - client connected.");
        connectionFailedCounter = 0;
      } 
      else {
        // if you didn't get a connection to the server:
        Serial.println("Ethernet Shield RT - No client found.");
        connectionFailedCounter++;
      }
    }
  }
  else {
    Serial.println("Ethernet Shield RT - Error.");

    if (client.connected()){
      // Disconnect from previous connection.
      client.stop();
    }

    InitialiseEthernetShield();
  }
}

void SaveCurrentPositionForStationKeeping() {
  holdX = 0;
  holdY = 0;
}

void HandleStationKeeping(){
  unsigned long now = millis();
  if ((now - lastMessageSent) > delayBetweenSendingMessagesMs)
  {
    lastMessageSent = now;

    if (stationKeepingEnabled) {
      RovInsReadMessage();
      //HandlePidAndSendThrusterCommand();
    }

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

void HandlePidAndSendThrusterCommand() {
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
  if (client.connected()){

    // TODO: Error checking with maintain();
    Ethernet.maintain();

    // if there are incoming bytes available
    // from the server, read them and print them:
    if (client.available()) {
      char c = client.read();
      if (c != '\n')
      {
        Serial.print(c);
      }
      else{
        Serial.println();
      }
    }
    else{
      delay(500);
    }
  }

  if (Serial.available()) {

    bool foundVelocityMessage = false;
    const int maxAttempts = 7;
    int counter = 0;

    char buffer[10];
    String message;
    do
    {
      message = Serial.readStringUntil(':');
      if (message.length() > 24)
      {
        if (message.charAt(0) == 'B' && message.charAt(1) == 'I' && message.charAt(2) == ',')
        {
          foundVelocityMessage = true;
        }
      }

      counter++;
    }
    while (Serial.available() && !foundVelocityMessage && counter < maxAttempts);
    
    if (foundVelocityMessage)
    {
      int xVelocity = message.substring(3, 9).toInt();
      int yVelocity = message.substring(10, 16).toInt();

      currentXAccumulation += xVelocity;
      currentYAccumulation += yVelocity;

      newVelocityData = true;

      Serial.println("Data: " + String(xVelocity) + "x, " + String(yVelocity) + "y.");
    }
  }
}