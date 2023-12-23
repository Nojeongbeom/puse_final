#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

const uint8_t DXL_IDS[] = {1, 2, 3, 4, 5, 6};  // List of Dynamixel IDs
const float DXL_PROTOCOL_VERSION = 2.0;

float dxl_goalPositions[] = {0,0,0,0,0,0};
float dxl_homePositions[] = {0,0,0,0,0,0};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  Serial.begin(9600);
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information for each motor
  for (uint8_t i = 0; i < sizeof(DXL_IDS) / sizeof(DXL_IDS[0]); i++) {
    dxl.ping(DXL_IDS[i]);
    dxl.torqueOff(DXL_IDS[i]);
    dxl.setOperatingMode(DXL_IDS[i], OP_EXTENDED_POSITION);
    dxl.torqueOn(DXL_IDS[i]);
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_IDS[i], 50);
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_IDS[i], 1000);
    dxl_homePositions[i] = dxl.readControlTableItem(PRESENT_POSITION, DXL_IDS[i]);
  }
}

void loop() {
  for (uint8_t i = 0; i < sizeof(DXL_IDS) / sizeof(DXL_IDS[0]); i++) {
    Serial.print("Pos for motor ");
    Serial.println(i+1);
    while(Serial.available() == 0){}
    float inputValue = Serial.readStringUntil('\n').toFloat();
    dxl_goalPositions[i] = deg2Pulse(inputValue);
    dxl.setGoalPosition(DXL_IDS[i], dxl_goalPositions[i] + dxl_homePositions[i]);
  }

  //Print current positions for each motor
  for (uint8_t i = 0; i < sizeof(DXL_IDS) / sizeof(DXL_IDS[0]); i++) {
    Serial.print("Motor ");
    Serial.print(DXL_IDS[i]);
    Serial.print(" POSITION DEGREE : ");
    Serial.println(dxl.getPresentPosition(DXL_IDS[i], UNIT_DEGREE));
  }
  Serial.println();  // Add a newline for clarity
}

float deg2Pulse(float deg){
  return deg / 360 * 4096;
}