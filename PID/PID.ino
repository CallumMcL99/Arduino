#include <DFRobot_MCP2515.h> // Can-bus
#include "DFRobot_BNO055.h" // Gyro
#include "Wire.h"
#include <PID.h>

#pragma region Variables
// Gyro variables
typedef DFRobot_BNO055_IIC Gyro;
Gyro _gyro(&Wire, 0x28);
Gyro::sAxisAnalog_t _startAxis;
bool _setStart = false;

// PID variables
double HeadingSetPoint, Input, Output, HeadingParallelSetPoint, HeadingNegativeOffset;
bool HeadingSetPointGreaterThanHalf;
double Kp=0, Ki=2, Kd=0;
arc::PID<double> HeadingPid(Kp, Ki, Kd);

// CAN-Bus variables
const int SPI_CS_PIN = 9;
DFRobot_MCP2515 CAN(SPI_CS_PIN);

uint32_t canID = 0x00FFFF0C;
const int size = 8;
uint8_t messageOne[size] =  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#pragma endregion Variables

#pragma region Setup
void setup()
{
  Serial.begin(115200);
  InitialiseGyro();
  InitialiseCanBus();
  SetHeadingSetPoint(100);
}

void InitialiseGyro()
{
  _gyro.reset();
  while (_gyro.begin() != Gyro::eStatusOK) {
    Serial.println("Gyro initialisation failed.");
    delay(2000);
  }

  Serial.println("Gyro initialised.");
}

void InitialiseCanBus(){
  bool canInit = false;
  do {
    if (CAN_OK != CAN.begin(CAN_500KBPS))
    {
        Serial.println("CAN shield failed to init.");
        delay(100);
    }
    else{
      canInit = true;
    }
  } while (!canInit);

}

#pragma endregion Setup

void loop()
{
  //ProcessCanBusMessages();
  GetLatitudeAndLongitude();
  delay(200);
}

#pragma region PID functions

#pragma region Heading
float GetRelativeHeading(){
  DFRobot_BNO055_IIC::sEulAnalog_t rotation = _gyro.getEul();
  float heading = rotation.head;
  
  if (HeadingSetPointGreaterThanHalf)
  {
    if (heading < HeadingParallelSetPoint)
    {
      heading += 360;
    }
  }
  else
  {
    if (heading > HeadingParallelSetPoint)
    {
      heading = -(360 - heading);
    }
  }

  // Serial.println("Current Heading: " + String(rotation.head));
  Serial.println("Relative Heading: " + String(heading));

  return heading;
}

void SetHeadingSetPoint(double setPoint){
  HeadingSetPoint = setPoint;
  HeadingPid.setTarget(setPoint);

  if (HeadingSetPoint > 180)
  {
    HeadingParallelSetPoint = HeadingSetPoint - 180;
    HeadingNegativeOffset = HeadingParallelSetPoint;
    HeadingSetPointGreaterThanHalf = true;
  }
  else
  {
    HeadingParallelSetPoint = HeadingSetPoint + 180;
    HeadingNegativeOffset = 360 - HeadingParallelSetPoint;
    HeadingSetPointGreaterThanHalf = false;
  }
}

void SetHeadingTunings(double kp, double ki, double kd)
{
  HeadingPid.setKp(kp);
  HeadingPid.setKi(ki);
  HeadingPid.setKd(kd);
}

void CalculateHeadingPid()
{
  if (_gyro.lastOperateStatus == Gyro::eStatusOK){
    float heading = GetRelativeHeading();

    HeadingPid.setInput(heading);

    Serial.println("PID Output: " + String(HeadingPid.getOutput()) + "\n");
  }
}
#pragma endregion Heading

void GetLatitudeAndLongitude()
{
  if (_setStart)
  {
    Gyro::sAxisAnalog_t  axis = _gyro.getAxis(Gyro::eAxisLia);
    Serial.println("x: " + String(axis.x - _startAxis.x) + ",\ty: " + String(axis.y- _startAxis.y) + ",\tz: " + String(axis.z - _startAxis.z));
    // Serial.println("x: " + String(axis.x) + ", y: " + String(axis.y) + ", z: " + String(axis.z));
    _startAxis = axis;
  }
  else
  {
    _setStart = true;
    _startAxis = _gyro.getAxis(Gyro::eAxisLia);
  }
}

#pragma endregion PID functions

#pragma region CAN-Bus functions
void ProcessCanBusMessages()
{
  if (CAN.checkReceive() == CAN_MSGAVAIL)
  {
    uint8_t id = 230;
    uint8_t len = 0;
    unsigned char buf[64];

    //CAN.readMsgBuf(&len, buf);
    if (CAN.readMsgBufID(id, &len, buf) == CAN_OK)
    {
      for(int i = 0; i < len; i++)
      {
          Serial.print(buf[i]);
          Serial.print("\t");
      }

      Serial.print("\n");
    }
  }
}

#pragma endregion CAN-Bus functions