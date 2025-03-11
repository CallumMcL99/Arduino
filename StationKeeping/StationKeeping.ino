#include <DFRobot_MCP2515.h> // Can Bus
#include "Wire.h"
#include <SPI.h>
#include <PID_v2.h>
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
const unsigned long delayBetweenSendingMessagesMs = 100;

// Station keeping
bool stationKeepingEnabled = false;
bool newPosition = false;
uint8_t yawMsb;
uint8_t yawLsb;
uint8_t verticalMsb;
uint8_t verticalLsb;
uint8_t foreMsb;
uint8_t foreLsb;
uint8_t latMsb;
uint8_t latLsb;

// Data from RovIns
double currentXVelocity;
double currentYVelocity;
bool newVelocityData = false;

// PID
//Define Variables we'll be connecting to
double xOutput, yOutput;
double xKp = 2, xKi = 5, xKd = 1;
double yKp = 2, yKi = 5, yKd = 1;
PID_v2 latitudePID(xKp, xKi, xKd, PID::Direct); //X
PID_v2 longitudePID(yKp, yKi, yKd, PID::Direct); //Y

// Ethernet
// This is the mac address and ip address for this device. The mac address is on the back of the Ethernet Shield.
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x24, 0x16
};
//IPAddress ip(192, 168, 137, 177);
IPAddress ip(192, 168, 36, 177);

// This port must match the port that tcp is being sent over.
// int port = 10002; // I use this port for simulation.
int ports[3] { 8111, 8112, 8113 };

// Enter the IP address of the server you're connecting to. This should match the comupters IPv4 address.
// If connecting to a Windows device, this is configured through:
// Contorl Panel > Network & Sharing Center > Ethernet 2 > Properties > TCP/IPv4 > IP address.
//IPAddress server(192, 168, 137, 1); // This is the IP used to connect the arduino to my PC for simulation.
IPAddress server(192, 168, 36, 135); // This is the IP used to connect to the RovIns. It's the same as the ID address for the web application. 192.168.36.1XX where XX is the last 2 digits of the serial number.

int connectionFailedCounters[3] = { 0, 0, 0 };
EthernetClient clients[3];
bool EthernetConnecteds[3] = { false, false, false };

// Attitude
double Pitch = 0;
double Roll = 0;
double Heading = 0;
double DistanceToBottom = 0;
double Altitude = 0;
double Depth;

// Debug
bool messageSent = false;
bool printFunctionEntry = false;

void setup() {
  Serial.begin(57600);
  while (!Serial) ;

  Serial.println("Serial initialisation - COMPLETE");

  InitialiseCanShield();
  InitialiseEthernetShield();

  latitudePID.SetOutputLimits(-1000, 1000);
  longitudePID.SetOutputLimits(-1000, 1000);
}

void InitialiseCanShield() {
  if (printFunctionEntry) Serial.println("ENTER: InitialiseCanShield");

  Serial.println("CAN Shield Initialisation - START");

  while (CAN_OK != CAN.begin(CAN_500KBPS)){
      Serial.println("CAN Shield Initialisation - ERROR");
      delay(100);
  }
  
  Serial.println("CAN Shield Initialisation - SUCCESS");
}

void InitialiseEthernetShield(){
  if (printFunctionEntry) Serial.println("ENTER: InitialiseEthernetShield");

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
  if (printFunctionEntry) Serial.println("ENTER: loop");
  
  bool client0Connected = clients[0].connected();

  // A delay seems to be needed between checking each client is connected, otherwise the latter check fails.
  delay(100);
  bool client1Connected = clients[1].connected();
  delay(100);
  bool client2Connected = clients[2].connected();

  if (client0Connected && client1Connected && client2Connected)
  {
    // TODO: Error checking with maintain();
    Ethernet.maintain();

    HandleCanBus();

    RovInsReadMessage_OctansStandard();
    //RovInsReadMessage_PhinsStandard(); // Not needed.
    RovInsReadMessage_RDIDP6();

    ProcessDataAndReply();
  }
  else
  {
    HandleEthernetShieldConnection(0);
    HandleEthernetShieldConnection(1);
    HandleEthernetShieldConnection(2);
  }
}

void HandleCanBus(){

  if (printFunctionEntry) Serial.println("ENTER: HandleCanBus");

  ReadCanBusMessage();
  // if (CAN.checkError() != CAN_CTRLERROR) {
  //   ReadCanBusMessage();
  // } else {
  //   Serial.println("Can bus RT - Error " + String(canErrorCount));
  //   canErrorCount++;

  //   if (canErrorCount > 30) {
  //     canErrorCount = 0;
  //     InitialiseCanShield();
  //   }
  // }
}

void ReadCanBusMessage() {
  
  if (printFunctionEntry) Serial.println("ENTER: ReadCanBusMessage");

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    
    uint8_t len = 0;
    while (CAN.checkReceive() == CAN_MSGAVAIL){
      uint8_t buf[16];

      CAN.readMsgBuf(&len, buf);

      uint32_t id = CAN.getCanId();
      uint32_t command = id >> 8;
      uint32_t address = id - (command << 8);

      //PrintCanMessage(id, buf, len);

      if (address == 9) {

        //PrintCanMessage(id, buf, len);
        if (command == 0xFF8E) {
          HandleSystemModeControlCommand(buf);
        }
        else if (command == 0xFFFC)
        {
          HandleSystemMotionControlCommand(buf);
        }
      }
      else if (command == 0xFFE7 && address == 0 && buf[0] == 4)
      {
        HandlePidTuningCommand(buf);
      }
    }
  }
}

void HandleSystemModeControlCommand(uint8_t buf[]){
  if (printFunctionEntry) Serial.println("ENTER: HandleSystemModeControlCommand");
  
  if (buf[5] == 1 || buf[5] == 2) {
    bool newStationKeeping = buf[5] == 2;

    // If value has been toggled.
    if (newStationKeeping != stationKeepingEnabled) {
      stationKeepingEnabled = newStationKeeping;

      if (stationKeepingEnabled) {     
        latitudePID = PID_v2(xKp, xKi, xKd, PID::Direct);
        longitudePID = PID_v2(yKp, yKi, yKd, PID::Direct);

        latitudePID.SetOutputLimits(-1000, 1000);
        longitudePID.SetOutputLimits(-1000, 1000);

        latitudePID.Start(0, 0, 0);
        longitudePID.Start(0, 0, 0);

        xOutput = 0;
        yOutput = 0;

        Serial.println("Station keeping enabled.");
        newPosition = false;
      } else {
        Serial.println("Station keeping disabled.");
      }
    }
  }
}

void HandlePidTuningCommand(uint8_t buf[]){
  if (printFunctionEntry) Serial.println("ENTER: HandlePidTuningCommand");

  xKp = buf[1];
  xKi = buf[2];
  xKd = buf[3];
  
  yKp = xKp;
  yKi = xKi;
  yKd = xKd;

  latitudePID.SetTunings(xKp, xKi, xKd);
  longitudePID.SetTunings(yKp, yKi, yKd);
  
  Serial.println("New tunings: p" + String(xKp) + ", i" + String(xKi) + ", d" + String(xKd));
}

void HandleSystemMotionControlCommand(uint8_t buf[]){
  if (printFunctionEntry) Serial.println("ENTER: HandleSystemMotionControlCommand");
  
  verticalMsb = buf[2];
  verticalLsb = buf[3];
  yawMsb = buf[4];
  yawLsb = buf[5];

}

void PrintCanMessage(uint32_t id, uint8_t buf[], uint8_t len){
  if (printFunctionEntry) Serial.println("ENTER: PrintCanMessage");

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
  if (printFunctionEntry) Serial.println("ENTER: SendCanMessage");

  uint32_t fullId = (id << 8) + address;
  CAN.sendMsgBuf(fullId, 1, size, message);
}

void HandleEthernetShieldConnection(int index){
  if (printFunctionEntry) Serial.println("ENTER: HandleEthernetShieldConnection");
  
  if (Ethernet.hardwareStatus() != EthernetNoHardware && Ethernet.linkStatus() != LinkOFF && connectionFailedCounters[index] < 20)
  {
    if (!clients[index].connected()){

      // Disconnect from previous connection.
      clients[index].stop();

      // if you get a connection, report back via serial:
      if (clients[index].connect(server, ports[index])) {
        Serial.println("Ethernet Shield RT " + String(index) + " - client connected.");
        EthernetConnecteds[index] = true;
        connectionFailedCounters[index] = 0;
      } 
      else {
        // if you didn't get a connection to the server:
        Serial.println("Ethernet Shield RT " + String(index) + " - No client found.");
        EthernetConnecteds[index] = false;
        connectionFailedCounters[index]++;
      }
    }
  }
  else {
    Serial.println("Ethernet Shield RT " + String(index) + " - Error.");
    EthernetConnecteds[index] = false;
    if (clients[index].connected()){
      // Disconnect from previous connection.
      clients[index].stop();
    }

    connectionFailedCounters[index] = 0;
    InitialiseEthernetShield();
  }
}

void ProcessDataAndReply(){
  if (printFunctionEntry) Serial.println("ENTER: HandleStationKeeping");

  unsigned long now = millis();
  if ((now - lastMessageSent) > delayBetweenSendingMessagesMs)
  {
    lastMessageSent = now;
    
    if (stationKeepingEnabled) {
      HandlePidAndSendThrusterCommand();
    }

    SendAttitudeMessage(Heading * 10, Pitch * 10, Roll * 10);
    SendDepthAltMessage(Depth * 100, Altitude * 10);
    SendHeartBeatMessage();

    if (!stationKeepingEnabled)
    {
      //Serial.println("Heartbeat: " + String(stationKeepingEnabled));
    }
    else
    {
      //Serial.println("Heartbeat: " + String(stationKeepingEnabled) + ". X: " + String(xOutput) + ". Y: " + String(yOutput));
    }
  }
}

void HandlePidAndSendThrusterCommand() {
  if (printFunctionEntry) Serial.println("ENTER: HandlePidAndSendThrusterCommand");

  if (newVelocityData)
  {
    newVelocityData = false;
    xOutput = latitudePID.Run(currentXVelocity);
    yOutput = longitudePID.Run(currentYVelocity);

    SendThrusterCommand(yOutput, xOutput);
    messageSent = true;
  }
}

void SendThrusterCommand(int forward, int lateral) {
  if (printFunctionEntry) Serial.println("ENTER: SendThrusterCommand");

  int lateralMsb = lateral >> 8;
  int forwardMsb = forward >> 8;

  if (lateral < 0 )
  {
    lateralMsb = 256 + lateralMsb;
  }

  if (forward < 0 )
  {
    forwardMsb = 256 + forwardMsb;
  }

  messageOne[0] = (byte)forwardMsb;
  messageOne[1] = (byte)forward & 0xFF;
  messageOne[2] = verticalMsb;
  messageOne[3] = verticalLsb;
  messageOne[4] = yawMsb;
  messageOne[5] = yawLsb;
  messageOne[6] = (byte)lateralMsb;
  messageOne[7] = (byte)lateral & 0xFF;

  Serial.println("Sent thruster command: F/A: " + String(forward) + ". Lateral: " + String(lateral) + ". Yaw: " + String(yawMsb) + " " + String(yawLsb));
  //Serial.println("Sent CMD: " + String(foreAft) + ", " + String(lateral) + ". Lat: " + String(currentLatitude) + " - " + String(holdLatitude));
  //Serial.println("Lateral: " + String(messageOne[6]) + " " + String(messageOne[7]));

  SendCanMessage(0xFFFC, 0, messageOne);
}

// Inputs expected from 0 to 3600 degrees
void SendAttitudeMessage(int heading, int pitch, int roll){
  if (printFunctionEntry) Serial.println("ENTER: SendAttitudeMessage");

  //pitch -= 900;
  //roll -= 900;

  int headingMsb = heading >> 8;
  int pitchMsb = pitch >> 8;
  int rollMsb = roll >> 8;

  if (heading < 0 )
  {
    headingMsb = 256 + headingMsb;
  }

  if (pitch < 0 )
  {
    pitchMsb = 256 + pitchMsb;
  }
  
  if (roll < 0 )
  {
    rollMsb = 256 + rollMsb;
  }

  messageOne[0] = (byte)headingMsb;
  messageOne[1] = (byte)heading & 0xFF;
  messageOne[2] = (byte)pitchMsb;
  messageOne[3] = (byte)pitch & 0xFF;
  messageOne[4] = (byte)rollMsb;
  messageOne[5] = (byte)roll & 0xFF;
  messageOne[6] = 0;
  messageOne[7] = 0;

  //Serial.println("Sent attitude: Heading: " + String(heading) + ", Pitch: " + String(pitch) + ", Roll: " + String(roll));
  SendCanMessage(0xFFFF, 0, messageOne);
}

void SendDepthAltMessage(int depth, int alt){
  if (printFunctionEntry) Serial.println("ENTER: SendDepthAltMessage");

  int depthMsb = depth >> 8;
  int altMsb = alt >> 8;

  if (depth < 0 )
  {
    depthMsb = 256 + depthMsb;
  }

  if (alt < 0 )
  {
    altMsb = 256 + altMsb;
  }

  messageOne[0] = (byte)depthMsb;       // depth msb
  messageOne[1] = (byte)depth & 0xFF;   // depth lsb
  messageOne[2] = (byte)altMsb;         // alt msb
  messageOne[3] = (byte)alt & 0xFF;     // alt lsb
  messageOne[4] = 0;
  messageOne[5] = 0;
  messageOne[6] = 0;
  messageOne[7] = 0;                    // depth 2 DP
  
  //Serial.println("Sent depth: Depth: " + String(depth) + ", Alt: " + String(alt));
  SendCanMessage(0xFFFE, 0, messageOne);
}

void SendHeartBeatMessage(){
  if (printFunctionEntry) Serial.println("ENTER: SendHeartBeatMessage");

    messageOne[0] = 0;
    messageOne[1] = 0;
    messageOne[2] = 0;
    messageOne[3] = 0;
    messageOne[4] = 0;
    messageOne[5] = stationKeepingEnabled ? 2 : 1;
    messageOne[6] = 0;
    messageOne[7] = 0;

    SendCanMessage(0xFF8F, 9, messageOne);
}

void RovInsReadMessage_OctansStandard() {
  if (printFunctionEntry) Serial.println("ENTER: RovInsReadMessage_OctansStandard");

  if (clients[0].available()) {

    // Here there are 2 messages to search for:
    // Heading - $HEHDT,4.55,T*1B
    // Pitch/Roll - $PHTRO,7.44,P,14.22,B*71

    bool foundHeadingMessage = false;
    bool foundPitchAndRollMessage = false;
    int counter = 0;
    while (clients[0].available() && counter++ < 8 && (!foundHeadingMessage || !foundPitchAndRollMessage))
    {
      String message = clients[0].readStringUntil('\n');
      //Serial.println(String(counter) + " - " + message);
        
      if (!foundHeadingMessage && message[3] == 'H' && message[4] == 'D' && message[5] == 'T')
      {
        // Heading message.
        // $HEHDT,4.55,T*1B

        char headingMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        bool commaFound = false;
        int j = 0;

        for (int i = 7; i < message.length() && !commaFound; i++)
        {
          if (message[i] != ',')
          {
            headingMessage[j] = message[i];
            j++;
          }
          else
          {
            commaFound = true;
          }
        }

        Heading = String(headingMessage).toFloat();
        foundHeadingMessage = true;
        //Serial.println("Heading: " + String(Heading));
      }
      else if (!foundPitchAndRollMessage && message[3] == 'T' && message[4] == 'R' && message[5] == 'O')
      {
        // Attitude message.
        // $PHTRO,7.44,P,14.22,B*71

        int commasFound = 0;
        int j = 0;
        char pitchMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        char rollMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        for (int i = 7; i < message.length() && commasFound < 3; i++)
        {
          if (message[i] != ',')
          {
            if (commasFound == 0)
            {
              pitchMessage[j] = message[i];
            }
            else if (commasFound == 2)
            {
              rollMessage[j] = message[i];
            }

            j++;
          }
          else
          {
            j = 0;
            commasFound++;
          }
        }

        Pitch = String(pitchMessage).toFloat();
        Roll = String(rollMessage).toFloat();
        foundPitchAndRollMessage = true;
        //Serial.println("Pitch: " + String(Pitch));
        //Serial.println("Roll: " + String(Roll));
      }
    }

    // Clear the buffer.
    // This is important as if a backlog of data builds up, we will never have current data, only exponentially old data.
    while (clients[0].available())
    {
      //Serial.println("Cleaing buffer");
      clients[0].readStringUntil('\n');
    }
  }
}

void RovInsReadMessage_PhinsStandard() {
  if (printFunctionEntry) Serial.println("ENTER: RovInsReadMessage_PhinsStandard");

  if (clients[1].available()) {

    // Search for the following messages, with these unique characters
    // $PIXSE,SPEED_,2.889,-4.202,-0.041*6B
    // $PIXSE,POSITI,-23.36996014,317.41925983,8.862*73

    bool speedMessageFound = false;
    bool positionMessageFound = false;
    int counter = 0;

    while (clients[1].available() && counter++ < 10 && (!speedMessageFound || !positionMessageFound))
    {
      String message = clients[1].readStringUntil('$');
      //Serial.print(String(counter) + " - " + message);
      
      if (message.length() > 12)
      {
        if (!speedMessageFound && message[8] == 'E'  && message[9] == 'E' && message[10] == 'D')
        {
          // SPEED - NOT IN USE
          // 0123456789
          // PIXSE,SPEED_,x.xxx,y.yyy,z.zzz*hh<CR><LF>
          
          int commasFound = 0;
          int j = 0;
          char speedXMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
          char speedYMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

          for (int i = 14; i < message.length() && commasFound < 3; i++)
          {
            if (message[i] != ',')
            {
              if (commasFound == 0)
              {
                speedXMessage[j] = message[i];
              }
              else if (commasFound == 2)
              {
                speedYMessage[j] = message[i];
              }

              j++;
            }
            else
            {
              j = 0;
              commasFound++;
            }
          }

          speedMessageFound = true;
          //currentXVelocity = String(speedXMessage).toFloat();
          //currentYVelocity = String(speedYMessage).toFloat();

          //Serial.println("X Velocity: " + String(currentXVelocity));
          //Serial.println("Y Velocity: " + String(currentYVelocity));
        }
        else if (false && !positionMessageFound && message[9] == 'I' && message[10] == 'T' && message[11] == 'I')
        {
          // NOT IN USE.
          // Position message.
          // PIXSE,POSITI,x.xxxxxxxx,y.yyyyyyyy,z.zzz*hh<CR><LF>
          
          int commasFound = 0;
          int j = 0;
          //char latMessage[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
          //char longMessage[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
          char altMessage[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

          for (int i = 13; i < message.length() && commasFound < 3; i++)
          {
            if (message[i] != ',' && message[i] != '*')
            {
              if (commasFound == 0)
              {
                //latMessage[j] = message[i];
              }
              else if (commasFound == 1)
              {
                //longMessage[j] = message[i];
              }
              else if (commasFound == 2)
              {
                altMessage[j] = message[i];
              }

              j++;
            }
            else
            {
              j = 0;
              commasFound++;
            }
          }
          
          positionMessageFound = true;
          Altitude = String(altMessage).toFloat();

          //Serial.println("Altitude: " + String(Altitude));
        }
      }
    }
    // Clear the buffer.
    // This is important as if a backlog of data builds up, we will never have current data, only exponentially old data.
    int cleanCounter = 0;
    while (clients[1].available())
    {
      cleanCounter++;
      clients[1].readStringUntil('\n');
    }

    if (cleanCounter != 0)
    {
      //Serial.println("Cleaned buffer: " + String(cleanCounter));
    }
  }
}

void RovInsReadMessage_RDIDP6() {
  if (printFunctionEntry) Serial.println("ENTER: RovInsReadMessage_RDIPD6");

  if (clients[2].available()) {

    // Search for the following messages, with these unique characters
    // $PIXSE,SPEED_,2.889,-4.202,-0.041*6B
    // $PIXSE,POSITI,-23.36996014,317.41925983,8.862*73

    bool speedMessageFound = false;
    bool bottomTrackMessage = false;
    bool depthMessage = false;
    int counter = 0;

    while (clients[2].available() && counter++ < 10 && (!speedMessageFound || !bottomTrackMessage))
    {
      String message = clients[2].readStringUntil(':');
      //Serial.println(String(counter) + " - " + message.length() + " - " + message);
      
      if (message.length() > 12)
      {
        if (!speedMessageFound && message[0] == 'B'  && message[1] == 'I' && message[2] == ',')
        {
          // Speed message
          // BI,±TTTTT,±LLLLL,±NNNNN,±MMMMM,S<CR><LF>

          speedMessageFound = true;
          newVelocityData = true;
          currentXVelocity = message.substring(3, 9).toFloat();
          currentYVelocity = message.substring(10, 16).toFloat();

          // For PID the values must be clamped to this range.
          if (currentXVelocity > 1023)
          {
            currentXVelocity = 1023;
          }
          else if (currentXVelocity < -1023)
          {
            currentXVelocity = -1023;
          }
          
          if (currentYVelocity > 1023)
          {
            currentYVelocity = 1024;
          }
          else if (currentYVelocity < -1023)
          {
            currentYVelocity = -1023;
          }

          //Serial.println("X Velocity: " + String(currentXVelocity));
          //Serial.println("Y Velocity: " + String(currentYVelocity));
        }
        else if (!bottomTrackMessage && message[0] == 'B' && message[1] == 'D' && message[2] == ',')
        {
          // Bottom track message.
          // BD,±EEEEEEEE.EE,±NNNNNNNN.NN,±UUUUUUUU.UU,DDDD.DD,TTT.TT <CR><LF>
          
          bottomTrackMessage = true;
          DistanceToBottom = message.substring(42, 49).toFloat();

          //Serial.println("Depth: " + String(Depth));
        }
        else if (!depthMessage && message[0] == 'T' && message[1] == 'S' && message[2] == ',')
        {
          // Depth message.
          depthMessage = true;
          Depth = message.substring(30, 35).toFloat();
          //Serial.println(String(Depth));
        }
      }
    }
    
    // Clear the buffer.
    // This is important as if a backlog of data builds up, we will never have current data, only exponentially old data.
    int cleanCounter = 0;
    while (clients[2].available())
    {
      cleanCounter++;
      clients[2].readStringUntil('\n');
    }

    if (cleanCounter != 0)
    {
      //Serial.println("Cleaned buffer: " + String(cleanCounter));
    }
  }
}