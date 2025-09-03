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
volatile double yKp = 0.2, yKi = 0, yKd = 0;
PID_v2 xPID(xKp, xKi, xKd, PID::Direct);
PID_v2 yPID(yKp, yKi, yKd, PID::Direct);
double accumX = 0;
double accumY = 0;
volatile long lastPidCalc = -1;

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
bool clientLastConnected[portCount] = {false, false};
String clientNames[portCount] = {"Cockpit", "DVL"};
bool EthernetConnecteds[portCount] = { false, false };
const int clientCockpitId = 0;
const int clientRdiPd6Id = 1;

volatile int pidRecievedCounter = 0;

float Altitude = 0;
volatile bool DvlReady = false; // The DVL won't send data if it's too close to the floor (50cm).

// Debug
bool printFunctionEntry = false;
bool printVersionOnLoop = false;
bool printDataRecv = false;
bool printDataSend = true;
bool printEthernet = true;


void setup() {
  //Serial.begin(250000, SERIAL_8N1);
  Serial.begin(9600, SERIAL_8N1);

  // Only use this while debugging. This waits for a serial (USB) connection.
  while (!Serial) ;

  Serial.println("Serial initialisation - COMPLETE");

  Serial.println(VERSION);
  InitialiseEthernetShield();

  xPID.SetSampleTime(100);
  yPID.SetSampleTime(100);

  int range = 100000;
  xPID.SetOutputLimits(-range, +range);
  yPID.SetOutputLimits(-range, +range);
  
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

bool client0Connected = false, client1Connected = false;
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

  if (Ethernet.linkStatus() != LinkOFF)
  {
    if (tick)
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
  
  if (client0Connected || client1Connected)
  {
    if (stationKeepingEnabled) {
      CalculatePids();
    }

    HandleSerialInput();

    SendDataToCockpit();
    // if (tick)
    // {
    // }
  }
  else
  {
    // If either connection is missing, sleep for 1 second. This gives the developer time to reprogram the arduino by stopping a connection.
    // (If an arduino is running code without a break it cannot be interupted to be reprogrammed)
    delay(1000);
  }
}

void SendDataToCockpit(){
  if (printFunctionEntry) Serial.println("ENTER: SendDataToCockpit");
  
  if (clients[clientCockpitId].connected())
  {
    // Serial.println("Ethernet Shield RT 0 - Write");
    EthernetClient client = clients[clientCockpitId];

    //$Ard_last DVL data ms,last pid,forward cmd,lateral cmd,forward vel, lateral val?
    String PostData = "$Ard_";
    PostData.concat(lastVelocityMessage);
    PostData.concat(",");
    PostData.concat(lastPidCalc);
    PostData.concat(",");
    PostData.concat(xOutput); // fwd cmd
    PostData.concat(",");
    PostData.concat(yOutput); // lat cmd
    PostData.concat(",");
    PostData.concat(currentXVelocity);
    PostData.concat(",");
    PostData.concat(currentYVelocity);
    PostData.concat("?");

    // client.println("POST /Api/AddParking/3 HTTP/1.1");
    // client.println("Host: 10.0.0.138");
    // client.println("User-Agent: Arduino/1.0");
    // // client.println("Connection: close");
    // client.print("Content-Length: ");
    // client.println(PostData.length());
    // client.println();
    client.print(PostData);

    if (printDataSend)
    {
      // Serial.println("SENDING: " + PostData);
    }
  }
}

void HandlePidTuningCommand(bool isX, float kP, float kI, float kD){
  if (printFunctionEntry) Serial.println("ENTER: HandlePidTuningCommand");

  if (isX)
  {
    xKp = kP;
    xKi = kI;
    xKd = kD;
    xPID.SetTunings(xKp, xKi, xKd);
    xPID.Start(accumX, 0, 0);
    xPID.SetMode(PID::Automatic);
  }
  else
  {
    yKp = kP;
    yKi = kI;
    yKd = kD;
    yPID.SetTunings(yKp, yKi, yKd);
    yPID.Start(accumY, 0, 0);
  }
}

void HandleEthernetShieldConnection(int index, bool rovIns){
  if (printFunctionEntry) Serial.println("ENTER: HandleEthernetShieldConnection for " + clientNames[index]);
  
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
          if (!clientLastConnected[index])
          {
            Serial.println("Ethernet Shield RT " + String(index) + " - client connected - " + clientNames[index]);
            clientLastConnected[index] = true;
          }

        }

        EthernetConnecteds[index] = true;
        connectionFailedCounters[index] = 0;
      } 
      else
      {
        // if you didn't get a connection to the server:
        if (printEthernet)
        {
          Serial.println("Ethernet Shield RT " + String(index) + " - No client found on " + server.toString() + ":" + String(port) + " - " + clientNames[index]);
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
      if (clientLastConnected[index])
      {
        Serial.println("Ethernet Shield RT " + String(index) + " - Error - " + clientNames[index]);
        clientLastConnected[index] = false;
      }
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

void HandleSerialInput()
{
  if (printFunctionEntry) Serial.println("ENTER: HandleSerialInput");
  
  if (Serial.available() > 0)
  {
    String s = Serial.readString();
    //Serial.print("READ: " + s);
    s.toLowerCase();

    if (s[0] == 'p' && s[1] == 'i' && s[2] == 'd')
    {
      if (s[3] != '?')
      {
        // pidx00.00,11.11,22.22
        String kP = s.substring(4, 9);
        String kI = s.substring(10, 15);
        String kD = s.substring(16, 21);

        Serial.print("PID parameters recv. ");
        if (s[3] == 'x')
        {
          Serial.print("X. ");
        }
        else if (s[3] == 'y')
        {
          Serial.print("Y. ");
        }
        Serial.println("kP: " + kP + ", kI: " + kI + ", kD: " + kD + ".");

        HandlePidTuningCommand(s[3] == 'x', kP.toFloat(), kI.toFloat(), kD.toFloat());
      }
      else
      {
        // pid?
        Serial.println("PID X. kP: " + String(xKp) + ", kI: " + String(xKi) + ", kD: " + String(xKd) + ".");
        Serial.println("PID Y. kP: " + String(yKp) + ", kI: " + String(yKi) + ", kD: " + String(yKd) + ".");
      }
    }
    else if (s[0] == 's' && s[1] == 'k')
    {
      //sk0
      ToggleStationKeeping(s[2] == '1');
    }
  }
}

void ToggleStationKeeping(bool enabled)
{
  if (printFunctionEntry) Serial.println("ENTER: ToggleStationKeeping");
  
  stationKeepingEnabled = enabled;
  
  currentYVelocity = 0;
  currentXVelocity = 0;
  accumY = 0;
  accumX = 0;
  xOutput = 0;
  yOutput = 0;

  if (stationKeepingEnabled)
  {
    Serial.println("Station keeping enabled.");
    
    yPID.Start(accumY, 0, 0);
    xPID.Start(accumX, 0, 0);
    
  }
  else
  {
    Serial.println("Station keeping disabled.");
  }
}

void CalculatePids() {
  if (printFunctionEntry) Serial.println("ENTER: HandlePidAndSendThrusterCommand");

  if (newVelocityData)
  {
    newVelocityData = false;
    accumX += currentXVelocity;
    accumY += currentYVelocity;
  }

  xOutput = xPID.Run(accumX);
  yOutput = yPID.Run(accumY);

  // xOutput = latitudePID.Run(currentXVelocity);
  // yOutput = longitudePID.Run(currentYVelocity);

  lastPidCalc = millis();

  if (printDataSend)
  {
    Serial.println(
      // "\nxVel: " + String(currentXVelocity) + ", xAccum: " + String(accumX) + ", xOut: " + String(xOutput) +
      "yVel: " + String(currentYVelocity) + ", yAccum: " + String(accumY) + ", yOut: " + String(yOutput)
      );
  }
}

const float badVelocityData = -32768.00;

void RovInsReadMessage_RDIDP6() {
  if (printFunctionEntry) Serial.println("ENTER: RovInsReadMessage_RDIPD6");

  if (clients[clientRdiPd6Id].available())
  {
    bool speedMessageFound = false;
    int counter = 0;

    // This must be greater than 4, otherwise it will not have time to cycle through all the previous messages.
    const int maxDvlReadAttempts = 10;
    
    while (clients[clientRdiPd6Id].available() && counter++ < maxDvlReadAttempts && !speedMessageFound)
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

          float xVel = message.substring(3, 9).toFloat();
          float yVel = message.substring(10, 16).toFloat();
          const float minRawVelocityReading = 1.5;

          if (xVel != badVelocityData && yVel != badVelocityData)
          {
            if (yVel > minRawVelocityReading || yVel < -minRawVelocityReading || xVel > minRawVelocityReading || xVel < -minRawVelocityReading)
            {
              newVelocityData = true;

              if (xVel > minRawVelocityReading || xVel < -minRawVelocityReading)
              {
                currentXVelocity = xVel / 100.0;
              }
              else
              {
                currentXVelocity = 0;
              }

              if (yVel > minRawVelocityReading || yVel < -minRawVelocityReading)
              {
                currentYVelocity = yVel / 100.0;
              }
              else
              {
                currentYVelocity = 0;
              }

              lastVelocityMessage = millis();

              if (printDataRecv)
              {
                Serial.println("Recv X Velocity: " + String(currentXVelocity) + ". Y Velocity: " + String(currentYVelocity));
                //Serial.println("Y Velocity: " + String(currentYVelocity));
              }
              else
              {
                Serial.println("Good velocity data recv.");
              }
              // For PID the values must be clamped to this range.
              // if (currentXVelocity > 1023)
              // {
              //   currentXVelocity = 1023;
              // }
              // else if (currentXVelocity < -1023)
              // {
              //   currentXVelocity = -1023;
              // }
              
              // if (currentYVelocity > 1023)
              // {
              //   currentYVelocity = 1024;
              // }
              // else if (currentYVelocity < -1023)
              // {
              //   currentYVelocity = -1023;
              // }
            }
            else
            {
              Serial.println("Tiny velocity data recv.");
            }
          }
          else{
            Serial.println("Bad velocity data recv.");
          }
        }
      }

      counter++;
    }

    
    if (!speedMessageFound)
    {
      Serial.println("No velocity data recv.");
    }


    // Clear the buffer.
    // This is important as if a backlog of data builds up, we will never have current data, only exponentially old data.
    int cleanCounter = 0;
    while (clients[clientRdiPd6Id].available())
    {
      cleanCounter++;
      //clients[clientRdiPd6Id].flush();
      // clients[clientRdiPd6Id].readStringUntil('\n');
      clients[clientRdiPd6Id].read();
    }

    if (printDataRecv && cleanCounter != 0)
    {
      Serial.println("Cleaned DVL: " + String(cleanCounter));
    }
  }

   //Serial.println("EXIT: RovInsReadMessage_RDIDP6");
}