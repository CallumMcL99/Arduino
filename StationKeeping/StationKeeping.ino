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
uint8_t foreMsb;
uint8_t foreLsb;
uint8_t latMsb;
uint8_t latLsb;

// Data from RovIns
double currentXVelocity;
double currentYVelocity;
double currentDepth;
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
int ports[2] { 8111, 8112 };

// Enter the IP address of the server you're connecting to. This should match the comupters IPv4 address.
// If connecting to a Windows device, this is configured through:
// Contorl Panel > Network & Sharing Center > Ethernet 2 > Properties > TCP/IPv4 > IP address.
//IPAddress server(192, 168, 137, 1); // This is the IP used to connect the arduino to my PC for simulation.
IPAddress server(192, 168, 36, 135); // This is the IP used to connect to the RovIns. It's the same as the ID address for the web application. 192.168.36.1XX where XX is the last 2 digits of the serial number.

int connectionFailedCounters[2] = { 0, 0 };
EthernetClient clients[2];
bool EthernetConnecteds[2] = { false, false };

// Attitude
double Pitch = 0;
double Roll = 0;
double Heading = 0;
double Depth = 0;
double Altitude = 0;

// Debug
bool messageSent = false;

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
  bool client0Connected = clients[0].connected();

  // A delay seems to be needed between checking each client is connected, otherwise the latter check fails.
  delay(100);
  bool client1Connected = clients[1].connected();

  if (client0Connected && client1Connected)
  {
    HandleCanBus();

    // Read messages as quickly as possible
    //RovInsReadMessage_OctansStandard();
    RovInsReadMessage_PhinsStandard();
    HandleStationKeeping();
  }
  else
  {
    HandleEthernetShieldConnection(0);
    HandleEthernetShieldConnection(1);
  }
}

void HandleCanBus(){
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
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    uint8_t len = 0;
    uint8_t buf[16];

    CAN.readMsgBuf(&len, buf);

    uint32_t id = CAN.getCanId();
    uint32_t command = id >> 8;
    uint32_t address = id - (command << 8);

    //PrintCanMessage(id, buf, len);

    if (address == arduinoAddress) {

      //PrintCanMessage(id, buf, len);

      if (command == 0xFFAA) {
        HandleSystemModeControlCommand(buf);
      }
      else if (command == 0xFF9D)
      {
        HandlePidTuningCommand(buf);
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
        latitudePID = PID_v2(xKp, xKi, xKd, PID::Direct);
        longitudePID = PID_v2(yKp, yKi, yKd, PID::Direct);

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
  xKp = buf[0];
  xKi = buf[1];
  xKd = buf[2];
  
  yKp = buf[3];
  yKi = buf[4];
  yKd = buf[5];

  latitudePID.SetTunings(xKp, xKi, xKd);
  longitudePID.SetTunings(yKp, yKi, yKd);
  
  Serial.println("New tunings: p" + String(buf[0]) + ", i" + String(buf[1]) + ", d" + String(buf[2]));
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

void HandleEthernetShieldConnection(int index){
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

void HandleStationKeeping(){
  unsigned long now = millis();
  if ((now - lastMessageSent) > delayBetweenSendingMessagesMs)
  {
    lastMessageSent = now;
    
    if (stationKeepingEnabled) {
      HandlePidAndSendThrusterCommand();
    }

    SendAttitudeMessage(Heading, Pitch, Roll);
    SendDepthAltMessage(Depth, Altitude);
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
  if (newVelocityData)
  {
    newVelocityData = false;
    xOutput = latitudePID.Run(currentXVelocity);
    yOutput = longitudePID.Run(currentYVelocity);

    Serial.println("PID processed: X in " + String(currentXVelocity) + ", x out " + String(xOutput) + ".");

    SendThrusterCommand(yOutput, xOutput);
    messageSent = true;
  }
}

void SendThrusterCommand(int forward, int lateral) {

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

  messageOne[0] = (byte)forwardMsb;//forward >> 8;    // Fore Msb
  messageOne[1] = (byte)forward & 0xFF;  // Fore Lsb
  messageOne[2] = 0;                     // Not used (vertical)
  messageOne[3] = 0;                     // Not used (vertical)
  messageOne[4] = 0;//yawMsb;
  messageOne[5] = 0;//yawLsb;
  messageOne[6] = (byte)lateralMsb;//(byte)lateral >> 8;    // Lateral Msb
  messageOne[7] = (byte)lateral & 0xFF;  // Lateral Lsb

  //Serial.println("Sent thruster command: F/A: " + String(forward) + ". Lateral: " + String(lateral) + ". Yaw: " + String(yawMsb) + " " + String(yawLsb));
  //Serial.println("Sent CMD: " + String(foreAft) + ", " + String(lateral) + ". Lat: " + String(currentLatitude) + " - " + String(holdLatitude));
  //Serial.println("Lateral: " + String(messageOne[6]) + " " + String(messageOne[7]));

  SendCanMessage(0xFFFC, 0, messageOne);
}

void SendAttitudeMessage(int heading, int pitch, int roll){
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

  messageOne[0] = (byte)depthMsb;
  messageOne[1] = (byte)depth & 0xFF;
  messageOne[2] = (byte)altMsb;
  messageOne[3] = (byte)alt & 0xFF;
  messageOne[4] = 0;
  messageOne[5] = 0;
  messageOne[6] = 0;
  messageOne[7] = 0;
  
  //Serial.println("Sent depth: Depth: " + String(depth) + ", Alt: " + String(alt));
  SendCanMessage(0xFFFE, 0, messageOne);
}

void SendHeartBeatMessage(){
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
  if (clients[0].connected()){

    // TODO: Error checking with maintain();
    Ethernet.maintain();

    if (clients[0].available()) {
      bool messageStarted = false;
      bool messageEnded = false;
      const int maxSize = 100;
      char message[maxSize];
      int i = 0;
      
      while(clients[0].available() && !messageEnded && i < maxSize)
      {
        char c = clients[0].read();
        if (messageStarted)
        {
          if (c != '\n')
          {
            message[i++] = c;
          }
          else
          {
            messageEnded = true;
          }
        }
        else
        {
          if (c == '$')
          {
            messageStarted = true;
          }
        }
      }

      if (messageEnded)
      {
        String messageStr = String(message);
        
        if (message[0] == 'H' && message[1] == 'E' && message[4] == 'T' && message[5] == ',')
        {
          //HEHDT,4.55,T*1B
          char headingMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
          bool commaFound = false;
          int j = 0;
          for (int i = 6; i < maxSize && !commaFound; i++)
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
          //Serial.println("Heading: " + String(Heading));
        }
        else if (message[0] == 'P' && message[1] == 'H' && message[4] == 'O' && message[5] == ',')
        {
          //PHTRO,7.44,P,14.22,B*71
          //Serial.println(messageStr);

          int commasFound = 0;
          int j = 0;
          char pitchMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
          char rollMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

          for (int i = 6; i < maxSize && commasFound < 3; i++)
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

          //Serial.println("Pitch: " + String(Pitch));
          //Serial.println("Roll: " + String(Roll));
        }
      }
      else if (!messageStarted)
      {
        //Serial.println("TCP message never started.");
      }
      else
      {
        //Serial.println("TCP message i: " + String(i));
      }
    }
  }
}

void RovInsReadMessage_PhinsStandard() {
  if (clients[1].connected()){

    // TODO: Error checking with maintain();
    Ethernet.maintain();

    if (clients[1].available()) {
      const int maxSize = 100;
      bool messageStarted = false;
      bool messageEnded = false;
      char message[maxSize];
      int i = 0;

      while(clients[1].available() && !messageEnded && i < maxSize)
      {
        char c = clients[1].read();
        if (messageStarted)
        {
          if (c != '\n')
          {
            message[i++] = c;
          }
          else
          {
            messageEnded = true;
          }
        }
        else
        {
          if (c == '$')
          {
            messageStarted = true;
          }
        }
      }

      if (messageEnded)
      {
        String messageStr = String(message);

        if (message[0] == 'P' && message[1] == 'I' && message[4] == 'E' && message[5] == ',')
        {
          if (message[6] == 'S' && message[7] == 'P' && message[10] == 'D' && message[11] == '_')
          {
            //$PIXSE,SPEED_,x.xxx,y.yyy,z.zzz*hh<CR><LF>
            //Serial.println(messageStr);
            //Serial.println("TCP message finished: " + messageStr);
            
            int commasFound = 0;
            int j = 0;
            char pitchMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            char rollMessage[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            for (int i = 13; i < maxSize && commasFound < 3; i++)
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

            //Serial.println("Pitch: " + String(Pitch));
            //Serial.println("Roll: " + String(Roll));




            // int xVelocity = messageStr.substring(3, 9).toInt();
            // int yVelocity = messageStr.substring(10, 16).toInt();

            // currentXVelocity = xVelocity;
            // currentYVelocity = yVelocity;

            // newVelocityData = true;

            //Serial.println("TCP Data: " + String(xVelocity) + "x, " + String(yVelocity) + "y.");
          }
          else if (message[6] == 'P' && message[7] == 'O' && message[11] == 'I' && message[12] == ',')
          {
            // $PIXSE,POSITI,x.xxxxxxxx,y.yyyyyyyy,z.zzz*hh<CR><LF>
            //Serial.println(messageStr);
            
            int commasFound = 0;
            int j = 0;
            char latMessage[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            char longMessage[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            char altMessage[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            for (int i = 13; i < maxSize && commasFound < 3; i++)
            {
              if (message[i] != ',' && message[i] != '*')
              {
                if (commasFound == 0)
                {
                  latMessage[j] = message[i];
                }
                else if (commasFound == 1)
                {
                  longMessage[j] = message[i];
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
            // Pitch = String(pitchMessage).toFloat();
            // Roll = String(rollMessage).toFloat();
            Altitude = String(altMessage).toFloat();

            //Serial.println("Pitch: " + String(Pitch));
            //Serial.println("Roll: " + String(Roll));
          }
        }
      }
      else if (!messageStarted)
      {
        //Serial.println("TCP message never started.");
      }
      else
      {
        //Serial.println("TCP message i: " + String(i));
      }
    }
  }
}