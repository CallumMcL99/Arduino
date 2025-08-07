/// ROV INS Arduino code. Version 1.0.1.
/// This handles communication with the RovIns and CAN BUS communication.
const String VERSION = " v1.0.2.5";

#include "Wire.h"
#include <SPI.h>
// #include "PID.h"
#include <PID_v2.h>
#include <Ethernet.h>

// Messages sent in the Heartbeat message to indicate what code is running.
const int InitialiseMessage_EthernetBegin = 1;
const int InitialiseMessage_EthernetCheckingForShield = 2;
const int InitialiseMessage_EthernetCheckingForCable = 3;
const int InitialiseMessage_EthernetFinished = 4;
const int InitialiseMessage_PidLimitsSet = 5;
const int InitialiseMessage_Complete = 6;
const int RunTimeMessage_Running = 7;

unsigned long delayBetweenSendingMessagesMs = 100;
unsigned long lastMessageSent = 0;

// Station keeping
volatile bool stationKeepingEnabled = false;
volatile bool newPosition = false;

// Data from RovIns
volatile double currentXVelocity;
volatile double currentYVelocity;
volatile bool newVelocityData = false;
volatile long lastVelocityMessage = -1;

// PID
//Define Variables we'll be connecting to
volatile double xOutput, yOutput;
volatile double xKp = 2, xKi = 5, xKd = 1;
volatile double yKp = 2, yKi = 5, yKd = 1;
PID_v2 latitudePID(xKp, xKi, xKd, PID::Direct); //X
PID_v2 longitudePID(yKp, yKi, yKd, PID::Direct); //Y
int accumX = 0;
int accumY = 0;

// Ethernet
// This is the mac address and ip address for this device. The mac address is on the back of the Ethernet Shield.
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x24, 0x16
};
//IPAddress ip(192, 168, 137, 177);
IPAddress ip(192, 168, 10, 174);

const int portCount = 2;
int ports[portCount] { 8111, 9004 };

// IPAddress server_rovIns(192, 168, 10, 201); // This is the IP used to connect to the RovIns. It's the same as the ID address for the web application. 192.168.36.1XX where XX is the last 2 digits of the serial number.
//                                   IP of PC,             IP of DVL
IPAddress servers[portCount] { IPAddress(192, 168, 10, 203), IPAddress(192, 168, 10, 201) };

// There are 3 connections: 2 listening to the RovinsNano (1 for each message), 1 for the Surface PC.
int connectionFailedCounters[portCount] = { 0, 0 };
EthernetClient clients[portCount];
bool EthernetConnecteds[portCount] = { false, false };
const int clientCockpitId = 0;
const int clientRdiPd6Id = 1;

volatile int pidRecievedCounter = 0;

float Altitude = 0;
volatile bool DvlReady = false; // The DVL won't send data if it's too close to the floor (50cm).

// Debug
bool printFunctionEntry = false;
bool printVersionOnLoop = false;
bool printDataRecv = true;
bool printDataSend = false;
bool printEthernet = true;


void setup() {
  //Serial.begin(250000, SERIAL_8N1);
  Serial.begin(9600, SERIAL_8N1);

  // Only use this while debugging. This waits for a serial (USB) connection.
  while (!Serial) ;

  Serial.println("Serial initialisation - COMPLETE");

  Serial.println(VERSION);
  InitialiseEthernetShield();

  latitudePID.SetOutputLimits(-1000, 1000);
  longitudePID.SetOutputLimits(-1000, 1000);
  
  Serial.println("Running " + VERSION);
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
    Serial.println("Ethernet Shield Initialisation - ERROR - cable is not connected x.");
    delay(500);
  }

  // Give the Ethernet shield a second to initialize.
  delay(1000);

  Serial.println("Ethernet Shield Initialisation - COMPLETE.");
}

void loop() {
  if (printFunctionEntry) Serial.println("ENTER: loop");
  
  if (printVersionOnLoop)
  {
    Serial.println("Running" + VERSION);
  }
  
  bool tick = false; 
  unsigned long now = millis();
  if ((now - lastMessageSent) > delayBetweenSendingMessagesMs)
  {
    lastMessageSent = now;
    tick = true;
  }

  bool client0Connected, client1Connected;
  if (Ethernet.linkStatus() != LinkOFF)
  {
    delay(100);
    client0Connected = clients[0].connected();

    // A delay seems to be needed between checking each client is connected, otherwise the latter check fails.
    delay(100);
    client1Connected = clients[1].connected();
    
    if (client0Connected || client1Connected)
    {
      Ethernet.maintain();
    }

    if (!client0Connected)
    {
      HandleEthernetShieldConnection(0, true);
    }

    if (client1Connected)
    {
      RovInsReadMessage_RDIDP6();
      DvlReady = true;
    }
    else
    {
      HandleEthernetShieldConnection(1, true);
      DvlReady = false;
    }
  }

  //ProcessDataAndReply(true);
  
  if (tick)
  {
    SendDataToCockpit();
  }
}

void SendDataToCockpit(){
  if (clients[clientCockpitId].connected())
  {
    // Serial.println("Ethernet Shield RT 0 - Write");
    EthernetClient client = clients[clientCockpitId];

    //$Ard_last DVL data ms,last pid,forward cmd,lateral cmd?
    String PostData = "$Ard_";// + "0" + "," + "0" + "," + "0" + "," + "0" + "?";

    client.println("POST /Api/AddParking/3 HTTP/1.1");
    client.println("Host: 10.0.0.138");
    client.println("User-Agent: Arduino/1.0");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(PostData.length());
    client.println();
    client.println(PostData);
  }
}

void HandlePidTuningCommand(uint8_t buf[], bool isX){
  if (printFunctionEntry) Serial.println("ENTER: HandlePidTuningCommand");

  if (isX)
  {
    xKp = buf[1];
    xKi = buf[2];
    xKd = buf[3];
    latitudePID.SetTunings(xKp, xKi, xKd);
    latitudePID.Start(0, 0, 0);
  }
  else
  {
    yKp = xKp;
    yKi = xKi;
    yKd = xKd;
    longitudePID.SetTunings(yKp, yKi, yKd);
    longitudePID.Start(0, 0, 0);
  }
  
  pidRecievedCounter = buf[4];

  String type;
  if (isX) type = "Lat";
  else type = "long";

  if (printDataRecv)
  {
    Serial.println("New tunings " + type + ": p" + String(xKp) + ", i" + String(xKi) + ", d" + String(xKd));
  }
}

void HandleEthernetShieldConnection(int index, bool rovIns){
  if (printFunctionEntry) Serial.println("ENTER: HandleEthernetShieldConnection");
  
  if (Ethernet.hardwareStatus() != EthernetNoHardware && Ethernet.linkStatus() != LinkOFF)// && connectionFailedCounters[index] < 20)
  {
    if (!clients[index].connected())
    {

      // Disconnect from previous connection.
      clients[index].stop();

      IPAddress server = servers[index];
      int port = ports[index];
      // if (rovIns) server = server_rovIns;
      //else server = server_surfacePc; 

      // if you get a connection, report back via serial:
      if (clients[index].connect(server, port))
      {
        if (printEthernet)
        {
          Serial.println("Ethernet Shield RT " + String(index) + " - client connected.");
        }

        EthernetConnecteds[index] = true;
        connectionFailedCounters[index] = 0;
      } 
      else
      {
        // if you didn't get a connection to the server:
        if (printEthernet)
        {
          Serial.println("Ethernet Shield RT " + String(index) + " - No client found on " + server.toString() + ":" + String(port) + ".");
        }
          
        EthernetConnecteds[index] = false;
        connectionFailedCounters[index]++;
      }
    }
  }
  else
  {
    if (printEthernet)
    {
      Serial.println("Ethernet Shield RT " + String(index) + " - Error.");
    }

    EthernetConnecteds[index] = false;
    if (clients[index].connected())
    {
      // Disconnect from previous connection.
      clients[index].stop();
    }

    connectionFailedCounters[index] = 0;
    InitialiseEthernetShield();
  }
}

void ProcessDataAndReply(bool tick){
  if (printFunctionEntry) Serial.println("ENTER: ProcessDataAndReply. Tick: " + String(tick));
    
  if (stationKeepingEnabled) {
    HandlePidAndSendThrusterCommand();
  }
}

void HandlePidAndSendThrusterCommand() {
  if (printFunctionEntry) Serial.println("ENTER: HandlePidAndSendThrusterCommand");

  if (newVelocityData)
  {
    newVelocityData = false;
    accumX += currentXVelocity;
    accumY += currentYVelocity;

    xOutput = latitudePID.Run(accumX);
    yOutput = longitudePID.Run(accumY);

    if (printDataSend)
    {
      Serial.println("xVel: " + String(currentXVelocity) + ", xOut: " + String(xOutput) + ", yVel: " + String(currentYVelocity) + ", yOut: " + String(yOutput));
    }
  }
}

void RovInsReadMessage_RDIDP6() {
  if (printFunctionEntry) Serial.println("ENTER: RovInsReadMessage_RDIPD6");

  if (clients[clientRdiPd6Id].available())
  {
    // Search for the following messages, with these unique characters
    // $PIXSE,SPEED_,2.889,-4.202,-0.041*6B
    // $PIXSE,POSITI,-23.36996014,317.41925983,8.862*73

    bool speedMessageFound = false;
    int counter = 0;

    while (clients[clientRdiPd6Id].available() && counter++ < 10 && speedMessageFound)
    {
      String message = clients[clientRdiPd6Id].readStringUntil(':');

      if (printDataRecv)
      {
        //Serial.println(message.length() + " - " + message);
      }
      
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

          lastVelocityMessage = millis();

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

          if (printDataRecv)
          {
            Serial.println("Recv X Velocity: " + String(currentXVelocity) + ". Y Velocity: " + String(currentYVelocity));
            //Serial.println("Y Velocity: " + String(currentYVelocity));
          }
        }
      }

      counter++;
    }

    // Clear the buffer.
    // This is important as if a backlog of data builds up, we will never have current data, only exponentially old data.
    int cleanCounter = 0;
    while (clients[clientRdiPd6Id].available())
    {
      cleanCounter++;
      clients[clientRdiPd6Id].readStringUntil('\n');
    }
  }
}