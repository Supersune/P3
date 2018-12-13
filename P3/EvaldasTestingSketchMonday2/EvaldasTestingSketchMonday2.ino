#include <SoftwareSerial.h>
#include <elapsedMillis.h>
const int BAUDRATE = 115200; //Sets the baudrate between the computer and the arduino.
// Aruduino ide doesn't like STX and ETX, so a and b for testing
const char STX = 'a';
const char ETX = 'b';
//const char STX = 0x02;
//const char ETX = 0x02;
const char SEP = ';'; //Sets the seperator between the servo angles sent form th ecomputer.
bool serialListen = false;
int counter = 0;

bool motorAngleSelect = false;
char motorSelect [2] = {0, 0};
char motorAngle[3] = {0, 0, 0};

String string = "";
int angle = 0;
int newAngle = 0;
const int MaxChars = 4;
char strValue[MaxChars+1];
//int index = 0;

//leds
int ledPin0 = 3;
int ledPin1 = 4;
int ledPin2 = 5;
int ledPin3 = 6;

const int motorCount = 5; //Sets how many servos you want to control.
int servoId [motorCount] = {1, 2, 3, 4, 5}; //Specifies the servo id's used.

//Defining joint limits

#define joint1_min_limit 0
#define joint1_max_limit 4095
#define joint2_min_limit 1927
#define joint2_max_limit 3350
#define joint3_min_limit 764
#define joint3_max_limit 3446

int command;
int posCap = 0;
float joint1Increment = 20;
float HeightIncrement = 10;
float stretchIncrement = 10;
float gripperIncrement = 20;
const float pi = 3.14159265358979323846;


//Test variables

//// Random initial joint values
////// Dynamixel step values

float posJoint[5];

//Kinematic variables

//// Forward

const float link1 = 25; //mm
const float link2 = 215; //mm
const float link3 = 260; //mm

float PX, PY, PZ; //Position of end-effector

const float QConversionConst = (2 * pi) / 4096; //Converting from Dynamixel steps/angles to radians. Radians per step.


//Servo servos [motorCount] = {};

#include <Arduino.h>
#include <math.h> //Equivalent of cmath

String input; 

// Using Arduino Mega? Uncomment this:
// #define ARDUINOMEMUSE

// Buffers for transmitting (tx) and receiving (rx).
// NB: If you transmit or receive packages longer than 30 bytes, then change the
// length definitions below.
#define TX_BUFFER_LEN 30
#define RX_BUFFER_LEN 30
uint8_t tx_buffer[TX_BUFFER_LEN];
uint8_t rx_buffer[RX_BUFFER_LEN];


// Low-level functions for setting individual bytes in the buffers.
void putInt8t(int8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

int8_t getInt8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void putUint8t(uint8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

uint8_t getUint8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void putInt16t(int16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

void putUint16t(uint16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

int16_t getInt16t(uint8_t* buffer, size_t pos)
{
  int16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

uint16_t getUint16t(uint8_t* buffer, size_t pos)
{
  uint16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

void putInt32t(int32_t val, uint8_t* buffer, size_t pos)
{
  for (int16_t i = 0; i < 4 ; i++) {
    buffer[pos + i] = (uint8_t)(val & 0x000000ff);
    val = val >> 8;
  }
}

int32_t getInt32t(uint8_t* buffer, size_t pos)
{
  int32_t v = 0;
  for (int16_t i = 3; i > -1 ; --i) {
    v = v << 8;
    v = v | (int32_t)buffer[pos + i];
  }
  return v;
}


// Dynamixel Protocol 2.0
// The protocol defines a header with fixed positions for the instruction and
// length fields:
#define DXL_LENGTH_POS 5
#define DXL_INSTRUCTION_POS 7

inline size_t getPackageLength(uint8_t* buffer)
{
  return getUint16t(buffer, DXL_LENGTH_POS);
}

#ifdef ARDUINOMEMUSE
const uint16_t  crc_table[] PROGMEM = {
#else
const uint16_t  crc_table[] = {
#endif

  0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
  0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
  0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
  0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
  0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
  0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
  0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
  0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
  0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
  0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
  0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
  0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
  0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
  0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
  0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
  0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
  0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
  0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
  0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
  0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
  0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
  0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
  0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
  0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
  0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
  0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
  0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};


uint16_t calculateCrc(uint16_t crc_accum, uint8_t* data_blk_ptr, size_t data_blk_size)
{
  size_t i, j;

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
#ifdef ARDUINOMEMUSE
    crc_accum = (crc_accum << 8) ^  pgm_read_word_near(crc_table + i);
#else
    crc_accum = (crc_accum << 8) ^ crc_table[i];
#endif
  }
  return crc_accum;
}

// Add CRC to a buffer
// Calculates the CRC and appends it to the buffer.
//
// @param buffer    The buffer to calculate CRC for.
// @param data_size The number of bytes in the buffer to calculate the CRC for.
//                  Incidentally, this is also the position in the array where
//                  the CRC will be added.
void addCrc(uint8_t* buffer, size_t data_size)
{
  uint16_t crc;
  crc = calculateCrc(0, buffer, data_size);
  putInt16t(crc, tx_buffer, data_size);
}

// Check the CRC of a package
// Calculates the CRC of the data in the buffer and compares it to the received
// CRC checksum.
//
// @param buffer the buffer holding the package.
// @param pos the position of the first CRC byte in the buffer.
// @return true if the CRC check is successful.
boolean checkCrc(uint8_t* buffer, int16_t pos)
{
  uint16_t incomming_crc = getUint16t(buffer, pos);
  uint16_t calculated_crc = calculateCrc(0, buffer, pos);
  return (calculated_crc == incomming_crc);
}

void setHdrAndID(uint8_t id) //Writes the Headders, reserved and the id to the transmission buffer.
{
  tx_buffer[0] = 0xff;
  tx_buffer[1] = 0xff;
  tx_buffer[2] = 0xfd;
  tx_buffer[3] = 0x00;
  tx_buffer[4] = id;
}


void dumpPackage(uint8_t *buffer) //Writes the buffer to the serial connectio to the computer.
{
  size_t l = getPackageLength(buffer) + 7;
  for (size_t i = 0; i < l; i++) {
    //Serial.print((int)buffer[i], HEX); Serial.print(" ");
  }
  //Serial.println("");
}

// Transmit the package in the tx_buffer
void transmitPackage()
{
  size_t pgk_length = getPackageLength(tx_buffer) + 7;
  Serial1.write(tx_buffer, pgk_length);
}

// Try to receive a package
//
// @param timeout milliseconds to wait for a reply.
// @returns true if a package was received and the CRC checks out.
bool receivePackage(size_t timeout = 100)
{
  elapsedMillis since_start = 0;
  size_t bytecount = 0;
  size_t remaining_read = 1;
  while (remaining_read > 0 && since_start < timeout)
  {
    if (Serial1.available())
    {
      uint8_t incomming_byte = Serial1.read();
      switch (bytecount)
      {
        case 0:
        case 1: if (incomming_byte == 0xFF) {
                  rx_buffer[bytecount] = incomming_byte;
                  ++bytecount;
                } else {
                  bytecount = 0;
                }
                break;
        case 2: if (incomming_byte == 0xFD)
                {
                  rx_buffer[bytecount] = incomming_byte;
                  ++bytecount;
                } else {
                  bytecount = 0;
                }
                break;
        case 3:
        case 4:
        case 5: rx_buffer[bytecount] = incomming_byte;
                ++bytecount;
                break;
        case 6: rx_buffer[bytecount] = incomming_byte;
                remaining_read = getPackageLength(rx_buffer);
                ++bytecount;
                break;
        default: rx_buffer[bytecount] = incomming_byte;
                 ++bytecount;
                 --remaining_read;
                 break;
      }
    }
  }
  if (remaining_read == 0)
  {
    //dumpPackage(rx_buffer, getPackageLength(rx_buffer));
    return checkCrc(rx_buffer, bytecount-2);
  }
  else
  {
    return false;
  }
}

// Send a READ instruction to the servo
void sendReadInstruction(uint8_t id, uint16_t from_addr, uint16_t data_length)
{
  /* Read instruction package
  0     1     2     3     4    5      6     7     8      9       10      11      12    13
  H1    H2    H3    RSRV  ID   LEN1   LEN2  INST  PARAM1 PARAM2  PARAM3  PARAM4  CRC1  CRC2
  0xFF  0xFF  0xFD  0x00  0x01  0x07  0x00  0x02  0x84   0x00    0x04    0x00    0x1D  0x15
  */
  setHdrAndID(id);
  putUint16t(7, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x02, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(from_addr, tx_buffer, 8);
  putUint16t(data_length, tx_buffer, 10);
  addCrc(tx_buffer, 12);

  transmitPackage();
}

// Send a 1 byte READ instruction to the servo
void sendWriteInstruction(uint8_t id, uint16_t address, uint8_t data)
{
  setHdrAndID(id);
  putUint16t(6, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putUint8t(data, tx_buffer, 10);
  addCrc(tx_buffer, 11);

  transmitPackage();
}

// Send a 4 byte READ instruction to the servo
void sendWriteInstruction(uint8_t id, uint16_t address, int32_t data)
{
  /* Read instruction package
  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
  H1   H2   H3   RSRV ID   LEN1 LEN2 INST P1   P2   P3   P4   P5   P6   CRC1 CRC2
  0xFF 0xFF 0xFD 0x00 0x01 0x09 0x00 0x03 0x74 0x00 0x00 0x02 0x00 0x00 0xCA 0x89
  */
  setHdrAndID(id);
  putUint16t(9, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putInt32t(data, tx_buffer, 10);
  addCrc(tx_buffer, 14);

  transmitPackage();
}


// Ping a servo
// Pings the servo and waits for a response. If none is received within the
// timeout, the ping fails. There is no check as to whether the response is
// appropriate. When sending a ping to broadcast (0xFE), several packages may be
// received. The user should look into the received package to see if the result
// is satisfactory.
//
// @param  id      the servo ID to ping.
// @param  timeout milliseconds before timing out.
// @return true if response is received.
bool doPing(uint8_t id, size_t timeout = 100)
{
  /* Ping Instruction Packet
    0     1     2     3     4     5     6     7     8     9
    H1    H2    H3    RSRV  ID    LEN1  LEN2  INST  CRC1  CRC2
    0xFF  0xFF  0xFD  0x00  0x01  0x03  0x00  0x01  0x19  0x4E
  */
  setHdrAndID(id);
  putUint16t(3, tx_buffer, DXL_LENGTH_POS); // length is at pos 5 and 6
  putUint8t(0x01, tx_buffer, DXL_INSTRUCTION_POS); // ping instruction (0x01) at pos 7
  addCrc(tx_buffer, 8); // CRC at pos 8 and 9
  transmitPackage();

  bool package_received = receivePackage(timeout);

  return package_received;
}

// Set the goal position of a servo
// Reads the position of a servo, if no response is read, the function does not
// modify the value of the position variable.
//
// @param id the servo ID to get the position of.
// @param position a reference to store the read position in.
// @return true if the servo responded.
bool readPosition(uint8_t id, int32_t& position)
{
  sendReadInstruction(id, 132, 4);
  if (receivePackage(1000))
  {
    position = getInt32t(rx_buffer, 9);
    return true;
  }
  else
  {
    return false;
  }
}

bool readCommand(String& serialInput)
{


    //serialInput = Serial.readStringUntil(';');
    return true;
  
}

// Read the position of a servo
//
// @param id       the servo ID to set the position of.
// @param position the position to go to.
// @return true if the servo responded.
bool setGoalPosition(uint8_t id, int32_t position)
{
  sendWriteInstruction(id, 116, position);
  if (receivePackage(1000))
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Enable torque on the servo
// The torque must be enabled for the motor to move.
//
// @param id     the servo ID to set the position of.
// @param enable whether the torque should be enabled or not.
// @return true if the servo responded.
bool torqueEnable(uint8_t id, bool enable)
{
  uint8_t enable_data = 0x01;
  if (! enable)
  {
    enable_data = 0x00;
  }
  sendWriteInstruction(id, 64, enable_data);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void moveToHomePos(){
  int32_t joint1HomePos =2048;
  int32_t joint2HomePos =3072;
  int32_t joint3HomePos =1024;
  int32_t joint4HomePos =2048;
  int32_t joint5HomePos =2048;

  setGoalPosition(1, joint1HomePos);
  setGoalPosition(2, joint2HomePos);
  setGoalPosition(3, joint3HomePos);
  setGoalPosition(4, joint4HomePos);
  setGoalPosition(5, joint5HomePos);
}

void sendAllMotorPositions(){
  int32_t intJointPos[5];

  for(int i=0; i<5; i++){
    intJointPos[i] =(int)posJoint[i];
    setGoalPosition(i+1, intJointPos[i]);
  }
}

void readAllJointPositions(){
  
  int32_t posRetrieve;
  
  for(int i=0; i<5; i++){
    readPosition(i+1, posRetrieve);
    posJoint[i] = posRetrieve;
   }
}

bool isDomain(float stepValue) {
  if (stepValue != stepValue) {
    return(false); //The stepValue is not in the workspace
  }
  else if (stepValue == stepValue) {
    return(true); //The stepValue is in the workspace
  }
}


float stepToRadians(float stepValue) {
  float radiansValue = (stepValue * QConversionConst)+pi;
  float twoPi = 2 * pi;
  if (radiansValue >= twoPi) {
    radiansValue = radiansValue - twoPi;
  }
  return(radiansValue);
}
float radiansToStep(float radiansValue) {
  float stepValue = (radiansValue + pi) / QConversionConst;
  if (stepValue >= 4095) {
    stepValue = stepValue - 4095;
  }
  return(stepValue);
}


bool joint2Limit(float jointValue) {
  if (jointValue <= joint2_max_limit) {
    if (jointValue >= joint2_min_limit) {
      return(true); //if the value is within the joint limit, return true.
    }
    else {
      return(false); //If the value is not within the joint limit, return current false.
    }
  }
  else {
    return(false); //If the value is not within the joint limit, return false.
  }

}

bool joint3Limit(float jointValue) {
  if (jointValue <= joint3_max_limit) {
    if (jointValue >= joint3_min_limit) {
      return(true); //if the value is within the joint limit return true.
    }
    else {
      return(false); //If the value is not within the joint limit, return false.
    }
  }
  else {
    return(false); //If the value is not within the joint limit, return false.
  }

}

float joint1Value(float currentJoint1Pos, float increment) {
  if (currentJoint1Pos+increment <= joint1_max_limit){
    if(currentJoint1Pos+increment >= joint1_min_limit){
      return(currentJoint1Pos + increment); //if the value is within the joint limit, increase the joint value by increment.
    }
    else {
      return(currentJoint1Pos); //If the value is not within the joint limit, return current joint value.
    }
  }
  else {
    return(currentJoint1Pos); //If the value is not within the joint limit, return current joint value.
  }
  
}

void forwardKinematicsXYZ(float motorPos1, float motorPos2, float motorPos3) {
  
  float Q1, Q2, Q3; //Theta 1-3

  Q1 = stepToRadians(motorPos1);
  Q2 = stepToRadians(motorPos2);
  Q3 = stepToRadians(motorPos3);

  PX = cos(Q1)*(link2*cos(Q2) + link3 * cos(Q2 + Q3));
  PY = sin(Q1)*(link2*cos(Q2) + link3 * cos(Q2 + Q3));
  PZ = link1 + link2*sin(Q2)+ link3 * sin(Q2 + Q3);
}

void HeightValue(float increment) {

  float Q1, Q2, Q3; //Theta 1-3
  float h, c3, s3, c1, s1, c2, s2, InvDeterminant; //A constant

  forwardKinematicsXYZ(posJoint[0], posJoint[1], posJoint[2]);
  PZ = PZ + increment;

  //Calculating theta 3

  h = pow(PX, 2) + pow(PY, 2) + pow((PZ - link1), 2);
  c3 = (h - pow(link2, 2) - pow(link3, 2)) / (2 * link2*link3); //cos theta 3
  s3 = -sqrt(1 - pow(c3, 2)); //sin theta 3 //changed to a minus
  Q3 = atan2(s3, c3);   

  //Calculating theta 1

  c1 = PX / sqrt(pow(PX, 2) + pow(PY, 2));
  s1 = PY / sqrt(pow(PX, 2) + pow(PY, 2));

  Q1 = atan2(PY, PX);

  //Calculating theta 2

  InvDeterminant = 1 / ((link2 + link3 * c3)*(link2 + link3 * c3) - (link3*s3)*(-link3 * s3)); //1/det(A)

  c2 = InvDeterminant * ((link2 + link3 * c3)*(c1*PX + s1 * PY) - (-link3 * s3)*(PZ-link1));
  s2 = InvDeterminant * (-(link3*s3)*(c1*PX + s1 * PY) + (link2 + link3 * c3)*(PZ - link1));
  
  Q2 = atan2(s2, c2);

  if (isDomain(radiansToStep(Q1)) == true)
  {

    if (      (isDomain(radiansToStep(Q2)) == true)   &&      (joint2Limit(radiansToStep(Q2)) == true))
    {

      if ((isDomain(radiansToStep(Q3)) == true) && (joint3Limit(radiansToStep(Q3)) == true))
      {
        posJoint[0] = radiansToStep(Q1);
        posJoint[1] = radiansToStep(Q2);
        posJoint[2] = radiansToStep(Q3);
      }
      else {
        //Serial.println("Value is out of bounds.");
      }
    }
    else {
        //Serial.println("Value is out of bounds.");
    }
  }
  else{
    //Serial.println("Value is out of bounds.");
  }
  
}


void stretch(float increment) {

  float diagonalXY, diagonalXYZ, c3, s3, Q2, Q3, sigma, E, lambda;
  forwardKinematicsXYZ(posJoint[0], posJoint[1], posJoint[2]); //updating XYZ positions


  diagonalXY = sqrt(pow(PX, 2) + pow(PY, 2)) + increment;
  diagonalXYZ = sqrt(pow((PZ-link1),2) + pow(diagonalXY,2));
  c3 = (pow(diagonalXYZ,2) - pow(link2, 2) - pow(link3, 2)) / (2 * link2*link3); //cos theta 3
  s3 = sqrt(1 - pow(c3, 2));

  Q3 = -atan2(s3, c3);

  sigma = atan2(PZ - link1, diagonalXY);
  E = link3 * s3; //internal variable
  lambda = asin(E / diagonalXYZ);

  Q2 = sigma + lambda;



    if ((isDomain(radiansToStep(Q2)) == true) && (joint2Limit(radiansToStep(Q2)) == true))
    {

      if ((isDomain(radiansToStep(Q3)) == true) && (joint3Limit(radiansToStep(Q3)) == true))
      {
        posJoint[1] = radiansToStep(Q2);
        posJoint[2] = radiansToStep(Q3);
      }
      else {
        //Serial.println("Value is out of bounds.");
      }
    }
    else {
      //Serial.println("Value is out of bounds.");
    }

}

void gripperCloseOpen(float increment)
{
  
    posJoint[3] = posJoint[3]+increment;
    posJoint[4] = posJoint[4]-increment;
  }


void kinematicMasterFunction(float userInput) {

  if(userInput == 3){
    posCap++;
  }
   else if(userInput == 4){
    --posCap;
  }
      if (posCap == 4) {
      posCap = 0;
    }
    if (posCap == -1){
      posCap = 3;
    }
    
    //Serial.println("Enter");
       if (posCap == 0) {
      posJoint[0] = joint1Value(posJoint[0], joint1Increment*userInput);
      digitalWrite(ledPin0,HIGH);
      digitalWrite(ledPin1,LOW);
      digitalWrite(ledPin2,LOW);
      digitalWrite(ledPin3,LOW);
      
    }

    else if (posCap == 1) {
        HeightValue(HeightIncrement*userInput);
      digitalWrite(ledPin0,LOW);
      digitalWrite(ledPin1,HIGH);
      digitalWrite(ledPin2,LOW);
      digitalWrite(ledPin3,LOW);
    }

    else if (posCap == 2) {
        stretch(stretchIncrement*userInput);
      digitalWrite(ledPin0,LOW);
      digitalWrite(ledPin1,LOW);
      digitalWrite(ledPin2,HIGH);
      digitalWrite(ledPin3,LOW);
    }

    else if (posCap == 3) {
        gripperCloseOpen(joint1Increment*userInput);
      digitalWrite(ledPin0,LOW);
      digitalWrite(ledPin1,LOW);
      digitalWrite(ledPin2,LOW);
      digitalWrite(ledPin3,HIGH);
    }


    sendAllMotorPositions();
}

int readToInt(char a){
  int intCommand = (int)a - 48;
  //Serial.print("Number of int: ");
  //Serial.println(intCommand);
  Serial.clear();
  return(intCommand);
}


int MaxBytesAvailable;
int CurrentAvailable;
int delayNumber=1000;
int n;
String messageOut;

// These are the setup/loop functions that Arduino require.
void setup()
{
//LED
pinMode(ledPin0, OUTPUT);
pinMode(ledPin1, OUTPUT);
pinMode(ledPin2, OUTPUT);
pinMode(ledPin3, OUTPUT);
  // Initialize USB serial port. (On Teensy, the baud rate is disregarded and is
  // always 12 Mbaud).
  //Serial.begin(BAUDRATE);
  Serial.begin(115200); //9600
  
  // Initialize the Dynamixel port (RX: pin0, TX: pin1, Transmit Enable: pin2.
  // With factory default baud rate of 57.6 kbaud).
  Serial1.begin(115200);
  Serial1.transmitterEnable(2);

  for (int i=0; i < motorCount; i++)//Pings all of the servos to make sure that we have a conection to all of the servos.
  {
    bool ping_success = doPing(servoId[i], 1000);
    //Serial.print("Ping: Servo #");
    dumpPackage(tx_buffer);
    //Serial.print(servoId[i]);
    //Serial.println(ping_success ? ", success!" : ", failure!");
    dumpPackage(rx_buffer);
  }
  if (true) //TODO: add condition that is false when a servo ping fails.
  {
    for (int i=1; i < motorCount+1; i++) //Enables torque for all servos. Can be simplified by using bulk write.
    {
     torqueEnable(i, true);
     //Serial.print("Torque Enable package sent: ");
     dumpPackage(tx_buffer);
     //Serial.print("Package received: ");
     dumpPackage(rx_buffer);
   }

 }

  moveToHomePos();
  readAllJointPositions();
  
}
void loop()
{
 // Serial.println("a");
  //delay(100);
/*
  bool gotStuff = 0;
  if(Serial.available()>0){gotStuff=1;}
  
  // put your main code here, to run repeatedly:
  while((gotStuff==1) && (Serial.available()<255)){
    char t = Serial.read();
    //Serial.println(Serial.read());
    Serial.println(t);
    Serial.flush();
    t = '0';
  }*/
  if(Serial.available()>0){
    
    string=(Serial.readString());
    kinematicMasterFunction(string.toFloat());
    //Serial.end();
   // Serial.println("b");
     Serial.clear();
      }
}
    
//}
    //input = "hej";
    //String anotherString = *inputString;
    //Serial.println(anotherString);
    //userInput= atoi(input.c_str());
/*
    if(input.equals("3")){
      //Serial.println(input);
      Serial.println("PosCap increased");
      posCap++;
    }
    */
    //Serial.flush();
  //delay(100);

  

//void serialEvent(){
  
    
    
    /*
        
          //If any input is detected in arduino
        
          String input;
          int userInput;
          char inputCommand;
          //int intCommand;
          inputCommand = Serial.read();
          //input = Serial.readStringUntil(';');
          //Serial.flush();
          
          int intCommand = (int)inputCommand - 48;
          //Serial.println(intCommand);
          switch(intCommand){
            case 1: Serial.write("You pressed 1");
            break;

            case 2: Serial.write("You pressed 2");
            break;

            //default: Serial.write("Default statement");
          }
      counter++;
      */

      //Serial.print("Counter: ");
      //Serial.println(counter);


          
            /*
          if(input.equals("3")){
            //Serial.println(input);
            //Serial.println("PosCap increased");
            posCap++;
          }
       
          if(input.equals("1")){
            //Serial.println(input);
            //Serial.println("Positive change.");
            Serial.write("You pressed 1");
            //kinematicMasterFunction(1);
            input="";
            }
          
          if(input.equals("2")){
            //Serial.println(input);
            //Serial.println("Negative change.");
            Serial.write("You pressed 2");
            //kinematicMasterFunction(2);
            input="";
            }
            */
           // Serial.flush();
           
        
//
