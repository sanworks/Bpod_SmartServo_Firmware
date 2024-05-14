/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_SmartServo_Firmware repository
  Copyright (C) Sanworks LLC, Rochester, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
// Smart Servo Module Firmware for Bpod
// Interfaces Dynamixel X series servos with MATLAB, Python and the Bpod State Machine.
// See https://robotis.us/x-series/
//
// Requirements:
// -Arduino 2.3.X or newer
// -Teensy board package v1.59.0 or newer, setup instructions at: https://www.pjrc.com/teensy/td_download.html
// -Robotis Dynamixel2Arduino library v0.7.0 or newer

#include <Dynamixel2Arduino.h>
#include "ArCOM.h"
#include "T4PortHandler.h"

// ---Macros---
// Pins
#define DXL_DIR_PIN1 12
#define DXL_DIR_PIN2 18
#define DXL_DIR_PIN3 19

// HW + FW versions
#define FIRMWARE_VERSION 1
#define HARDWARE_VERSION 1

// Motor programs
#define MAX_STEPS 256
#define MAX_PROGRAMS 100

// Motor properties
#define MAX_MOTOR_ADDR 3

// Module properties
char moduleName[] = "Servo"; 
char* eventNames[] = {"Ch1M1Run", "Ch1M1End", "Ch1M2Run", "Ch1M2End", "Ch1M3Run", "Ch1M3End",
                      "Ch2M1Run", "Ch2M1End", "Ch2M2Run", "Ch2M2End", "Ch2M3Run", "Ch2M3End",
                      "Ch3M1Run", "Ch3M1End", "Ch3M2Run", "Ch3M2End", "Ch3M3Run", "Ch3M3End"};
byte nEventNames = (sizeof(eventNames)/sizeof(char *));
byte eventCode[3][3][2] = { 
                     { {1, 2}, {3, 4}, {5, 6} },
                     { {7, 8}, {9, 10}, {11, 12} },
                     { {13, 14}, {15, 16}, {17, 18} } };

const byte dioPins[3] = {9,8,7};
const byte circuitRevisionArray[5] = {2,3,4,5,6};

// Namespace (for Dynamixel control table items)
using namespace ControlTableItem;  

// Setup interfaces
ArCOM USBCOM(Serial);
ArCOM StateMachineCOM(Serial1);
byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer
Dynamixel2Arduino dxl1;
T4PortHandler dxl_port1(Serial3, DXL_DIR_PIN1);
byte Serial3Buf[192] = {0}; // Extra memory for port 1 serial buffer
Dynamixel2Arduino dxl2;
T4PortHandler dxl_port2(Serial4, DXL_DIR_PIN2);
byte Serial4Buf[192] = {0}; // Extra memory for port 2 serial buffer
Dynamixel2Arduino dxl3;
T4PortHandler dxl_port3(Serial5, DXL_DIR_PIN3);
byte Serial5Buf[192] = {0}; // Extra memory for port 3 serial buffer

// Hardware timer
IntervalTimer hardwareTimer; // Hardware timer to create even sampling

// Constants
const uint32_t validBaudRates[7] = {4000000, 57600, 1000000, 2000000, 3000000, 115200, 9600}; // Ordered by most likely encounters during auto-detection
const float vel2Units = 262.08; // Conversion of revolutions per second to Dynamixel's internal profile velocity units
const float acc2Units = 60/214.577; // Conversion of revolutions per second^2 to Dynamixel's internal profile acceleration units
const float current2Units = 1; // Conversion of mA to Dynamixel's internal current limit units

// Motor program metadata
bool program_loaded[MAX_PROGRAMS] = {false}; // true for each program if a motor program was loaded via USB
volatile bool program_running[MAX_PROGRAMS] = {false}; // true for each program if the program was triggered and is actively executing
uint8_t program_origin[MAX_PROGRAMS] = {0}; // Indicates which serial interface triggered the program 0 = USB 1 = State Machine
uint8_t program_nSteps[MAX_PROGRAMS] = {0}; // Number of steps in each program
                                         //               1 = movements defined by position/motor current limit
uint8_t program_moveType[MAX_PROGRAMS] = {0}; // Move type for each instruction in each motor program. 0 = velocity limited, 1 = current limited 
uint8_t program_currentStep[MAX_PROGRAMS] = {0}; // Tracks the current step in each program as it is executed
uint32_t program_currentTime[MAX_PROGRAMS] = {0}; // Tracks the current time with respect to program start as each program is executed
uint32_t program_loopCount[MAX_PROGRAMS] = {0}; // Tracks the current time with respect to trigger start as each looping program is executed

// Motor program content
uint8_t program_channel[MAX_PROGRAMS][MAX_STEPS] = {0}; // Motor channel for each instruction in each motor program
uint8_t program_address[MAX_PROGRAMS][MAX_STEPS] = {0}; // Motor address for each instruction in each motor program
DMAMEM float program_goalPosition[MAX_PROGRAMS][MAX_STEPS] = {0}; // Goal position for each instruction in each motor program
DMAMEM float program_goalMovementLimit[MAX_PROGRAMS][MAX_STEPS] = {0}; // Max velocity (when program_MoveType = 0)  or current limit (when program_MoveType = 1) 
uint32_t program_stepTime[MAX_PROGRAMS][MAX_STEPS] = {0}; // The time to execute each step in each motor program, with respect to program start
uint32_t program_nLoops[MAX_PROGRAMS] = {0}; // If >0, the program loops for a given amount of time

// Structure for type conversion
union {
    byte byteArray[4];
    uint16_t uint16;
    uint32_t uint32;
    int8_t int8;
    int16_t int16;
    int32_t int32;
    float floatVal;
} typeBuffer;

// DIO parameters
uint8_t dio_FallingEdgeFcn[3] = {0}; // 0 = No Function, 1 = Start Target Program, 2 = Stop Target Program, 3 = Emergency Stop-All
uint8_t dio_RisingEdgeFcn[3] = {0}; // 0 = No Function, 1 = Start Target Program, 2 = Stop Target Program, 3 = Emergency Stop-All
uint8_t dio_TargetProgram[3] = {0}; // Target motor program for each DIO channel 
uint32_t dio_Debounce[3] = {100, 100, 100}; // Debounce interval (units = multiples of the 100us HW timer interval)

// General variables
bool motorDetected[3][MAX_MOTOR_ADDR] = {false}; // Set to true if motor is detected
uint8_t motorMode[3][MAX_MOTOR_ADDR] = {0}; // 1 = Position, 2 = Extended Position, 3 = Current-Limited Position, 4 = Velocity, 5 = Step
float currentPosition[3][MAX_MOTOR_ADDR] = {0}; // Last known position
bool trackMovement[3][MAX_MOTOR_ADDR] = {false}; // True if tracking a movement, to send a behavior event to the FSM when the movement ends
uint8_t trackMovementSource[3][MAX_MOTOR_ADDR] = {0}; // Source serial interface of track movement (same as opSource, 0 = USB, 1 = State Machine)
float trackMovementGoal[3][MAX_MOTOR_ADDR] = {0}; // Goal position of tracked movement
uint8_t currentDioState[3] = {0}; // Current state of DIO lines
uint8_t lastDioState[3] = {0}; // Last known DIO state
uint32_t debounceCounter[3] = {0}; // count of hardware timer calls since debounce period started
bool inDebounce[3] = {0}; // true if in debounce interval, false if not
uint8_t address = 1; // Motor address on a channel (1-253)
uint8_t channel = 1; // Hardware serial channel targeted (1-3)
uint8_t newMode = 1; // Motor mode
uint8_t newID = 0; // New motor address
uint8_t programID = 0; // ID of the current program
uint8_t moveType = 0; // Move type of the current program (0 = position/velocity/acceleration, 1 = position/motor current)
uint8_t tableIndex = 0; // Control table index
uint8_t opCode = 0; // Op code, a byte code to identify each operation
uint8_t opSource = 0; // Op source, 0 = USB, 1 = State Machine
uint8_t focusChannel = 0; // The channel currently in focus
uint8_t focusAddress = 0; // The motor address currently in focus
uint8_t blocking = 0; // 0: confirm new position goal immediately 1: confirm after movement finishes executing
uint8_t result = 0; // Result of an op (1 = success, 0 = fail)
uint32_t nSteps = 0; 
float value = 0; // Temporary float
float maxVelocity = 0; // Maximum velocity (Rev / Second)
float maxAccel = 0; // Maximum acceleration
float pos = 0; // Position (degrees)
float current = 0; // Current limit (percent)
float newVelocity = 0; // Velocity (Rev / Second)
float newPosition= 0;
float thisMovementGoal = 0;
uint8_t circuitRevision = 0;
uint32_t monitorStartTime = 0; 


void setup() {
  // Initialize hardware serial interfaces
  Serial1.begin(1312500); // State machine
  Serial1.addMemoryForRead(StateMachineSerialBuf, 192);
  Serial3.begin(4000000); // Motor channel 1
  Serial3.transmitterEnable(DXL_DIR_PIN1);
  Serial3.addMemoryForRead(Serial3Buf, 192);
  Serial4.begin(4000000); // Motor channel 2
  Serial4.transmitterEnable(DXL_DIR_PIN2);
  Serial4.addMemoryForRead(Serial4Buf, 192);
  Serial5.begin(4000000); // Motor channel 3
  Serial5.transmitterEnable(DXL_DIR_PIN3);
  Serial5.addMemoryForRead(Serial5Buf, 192);

  // Setup motor channels
  dxl1.setPort(dxl_port1);
  dxl1.begin(4000000);
  dxl1.setPortProtocolVersion(2.0);
  dxl2.setPort(dxl_port2);
  dxl2.begin(4000000);
  dxl2.setPortProtocolVersion(2.0);
  dxl3.setPort(dxl_port3);
  dxl3.begin(4000000);
  dxl3.setPortProtocolVersion(2.0);

  // Initialize DIO pins as input (high impedance)
  for (int i = 0; i < 3; i++) {
    pinMode(dioPins[i], INPUT_PULLUP);
  }

  // Read hardware revision from circuit board (an array of grounded pins indicates revision in binary, grounded = 1, floating = 0)
  circuitRevision = 0;
  for (int i = 0; i < 5; i++) {
    pinMode(circuitRevisionArray[i], INPUT_PULLUP);
    delay(1);
    circuitRevision += pow(2, i)*digitalRead(circuitRevisionArray[i]);
    pinMode(circuitRevisionArray[i], INPUT);
  }
  circuitRevision = 31-circuitRevision; 
  monitorStartTime = micros();
  // Start hardware timer
  hardwareTimer.begin(programHandler, 1000);
}

void loop() {
  opCode = 0;
  if (StateMachineCOM.available() > 0) {
      opCode = StateMachineCOM.readByte();
      opSource = 1;
  } else if (USBCOM.available() > 0) {
      opCode = USBCOM.readByte();
      if (opCode == 212) { // Menu access byte via USB, to lower the odds of interference from other programs connecting to the port
        opCode = USBCOM.readByte();
        opSource = 0;
      } else {
        opCode = 0;
      }
  }
  if (opCode > 0) {
    switch(opCode) {
      case 255: // Return Bpod module info
        if (opSource == 1) { // Only returns this info if requested from state machine device
          returnModuleInfo();
        }
      break;

      case 249: // Handshake and set defaults for new session
        if (opSource == 0) {
          USBCOM.writeByte(250);
          clearMotorPrograms();
        }
      break;

      case 'D': // Discover motors, initialize and return metadata
        if (opSource == 0) {
          discoverMotors(true); // Arg: true returns metadata via USB, false does not
        }
      break;

      case '?': // Return module information
        if (opSource == 0) {
          USBCOM.writeUint32(FIRMWARE_VERSION);
          USBCOM.writeUint32(HARDWARE_VERSION);
          USBCOM.writeUint32(MAX_PROGRAMS);
          USBCOM.writeUint32(MAX_STEPS);
        }
      break;

      case '[': // Set max velocity
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        maxVelocity = readFloatFromSource(opSource);
        writeMaxVelocity_Fast(channel, address, maxVelocity);
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
      break;

      case ']': // Set max acceleration
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        maxAccel = readFloatFromSource(opSource);
        writeMaxAcceleration_Fast(channel, address, maxAccel);
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
      break;

      case 'F': // Set channel and address of motor in focus, for focused movement commands
        focusChannel = readByteFromSource(opSource);
        focusAddress = readByteFromSource(opSource);
        if (opSource == 0) {
          USBCOM.writeByte(1);
        }
      break;

      case 'P': // Set position goal of a target motor, at current velocity and acceleration
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        setGoalPosition(channel, address, pos);
      break;

      case '>': // Set position of the motor currently in focus, at current velocity and acceleration
        pos = readFloatFromSource(opSource);
        if (focusChannel > 0 && focusAddress  > 0) {
          setGoalPosition(focusChannel, focusAddress, pos);
        }
      break;

      case 'G': // Set position goal with a new velocity and acceleration
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        blocking = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        maxVelocity = readFloatFromSource(opSource);
        maxAccel = readFloatFromSource(opSource);
        if ((motorMode[channel-1][address-1] == 1) || (motorMode[channel-1][address-1] == 2)) {
          writeMaxVelocity_Fast(channel, address, maxVelocity);
          writeMaxAcceleration_Fast(channel, address, maxAccel);
          setGoalPosition(channel, address, pos);
          if (blocking) {
            delayUntilMovementEnd(channel, address, pos);
            if (opSource == 0) {
              USBCOM.writeByte(1);
            }
          }
        } else {
          if (opSource == 0) {
            USBCOM.writeByte(0);
          }
        }

      break;

      case 'C': // Set current-limited position goal
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        current = readFloatFromSource(opSource);
        if (motorMode[channel-1][address-1] == 3) {
          writeMaxCurrent_Fast(channel, address, current);
          setGoalPosition(channel, address, pos);
        } else {
          if (opSource == 0) {
            USBCOM.writeByte(0);
          }
        }
      break;

      case 'V': // Set velocity goal (in motor mode 4). Units = rev/s. Sign indicates direction.
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        newVelocity = readFloatFromSource(opSource);
        if (motorMode[channel-1][address-1] == 4) {
          writeGoalVelocity_Fast(channel, address, newVelocity);
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
        } else {
          if (opSource == 0) {
            USBCOM.writeByte(0);
          }
        }
      break;

      case 'S': // Step a given distance from the current shaft position (in motor mode 5) Units = degrees. Sign indicates direction.
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        newPosition = currentPosition[channel-1][address-1]+pos;
        stepMotor(channel, address, newPosition);
      break;

      case '^': // Step a given distance using the motor currently in focus, at current velocity and acceleration
        pos = readFloatFromSource(opSource);
        newPosition = currentPosition[channel][address]+pos;
        if (focusChannel > 0 && focusAddress  > 0) {
          stepMotor(focusChannel, focusAddress, newPosition);
        }
      break;

      case 'L': // Load a motor program
        if (opSource == 0) {
          programID = USBCOM.readByte();
          nSteps = USBCOM.readByte();
          program_nSteps[programID] = nSteps;
          program_moveType[programID] = USBCOM.readByte();
          program_nLoops[programID] = USBCOM.readUint32();
          for (int i = 0; i < nSteps; i++) {
            program_channel[programID][i] = USBCOM.readByte();
          }
          for (int i = 0; i < nSteps; i++) {
            program_address[programID][i] = USBCOM.readByte();
          }
          for (int i = 0; i < nSteps; i++) {
            program_goalPosition[programID][i] = USBCOM.readFloat();
          }
          for (int i = 0; i < nSteps; i++) {
            program_goalMovementLimit[programID][i] = USBCOM.readFloat();
          }
          for (int i = 0; i < nSteps; i++) {
            program_stepTime[programID][i] = USBCOM.readUint32();
          }
          program_loaded[programID] = true;
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case 'R': // Run motor program
        programID = readByteFromSource(opSource);
        startProgram(programID, opSource);
      break;

      case '=': // Set target motor program for each DIO channel 
        if (opSource == 0) {
          USBCOM.readByteArray(dio_TargetProgram, 3);
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case '+': // Set operation on DIO rising edge,   0 = No Operation, 
                                                    // 1 = Start Target Program
                                                    // 2 = Stop Target Program after current move
                                                    // 3 = Emergency Stop: Torque --> 0 for all motors, stop all programs.
                                                    //     Torque can be re-enabled by setting the motor mode for each motor.
        if (opSource == 0) {
          USBCOM.readByteArray(dio_RisingEdgeFcn, 3);
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case '-': // Set operation on DIO falling edge, Same codes as op '+' above
        if (opSource == 0) {
          USBCOM.readByteArray(dio_FallingEdgeFcn, 3);
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case '~': // Set debounce interval (units = multiples of the 100us HW timer interva
        if (opSource == 0) {
          USBCOM.readUint32Array(dio_Debounce, 3);
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case '!': // Emergency stop
        emergency_stop();
        if (opSource == 0) {
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case '%': // Return current position via USB
        if (opSource == 0) {
          channel = USBCOM.readByte();
          address = USBCOM.readByte();
            switch(channel) {
              case 1:
                pos = dxl1.getPresentPosition(address, UNIT_DEGREE);
              break;
              case 2:
                pos = dxl2.getPresentPosition(address, UNIT_DEGREE);
              break;
              case 3:
                pos = dxl3.getPresentPosition(address, UNIT_DEGREE);
              break;
            }
            USBCOM.writeFloat(pos);
        }
      break;

      case 'T': // Return value from control table via USB
        if (opSource == 0) {
          channel = USBCOM.readByte();
          address = USBCOM.readByte();
          tableIndex = USBCOM.readByte();
          switch(channel) {
            case 1:
              USBCOM.writeInt32(dxl1.readControlTableItem(tableIndex, address));
            break;
            case 2:
              USBCOM.writeInt32(dxl2.readControlTableItem(tableIndex, address));
            break;
            case 3:
              USBCOM.writeInt32(dxl3.readControlTableItem(tableIndex, address));
            break;
          }
        }
      break;

      case 'M': // Set motor mode
        if (opSource == 0) {
          channel = USBCOM.readByte();
          address = USBCOM.readByte();
          newMode = USBCOM.readByte();
          setMotorMode(channel, address, newMode);
          if (newMode == 5) {
            switch(channel) {
              case 1:
                currentPosition[channel-1][address-1] = dxl1.getPresentPosition(address, UNIT_DEGREE);
              break;
              case 2:
                currentPosition[channel-1][address-1] = dxl2.getPresentPosition(address, UNIT_DEGREE);
              break;
              case 3:
                currentPosition[channel-1][address-1] = dxl3.getPresentPosition(address, UNIT_DEGREE);
              break;
            }
          }
          USBCOM.writeByte(1);
        }
      break;

      case 'I': // Set motor ID (address on channel). Written to motor EEPROM for all future sessions.
        if (opSource == 0) {
          channel = USBCOM.readByte();
          address = USBCOM.readByte();
          newID = USBCOM.readByte();
          switch(channel) {
            case 1:
              dxl1.torqueOff(address);
              result = dxl1.setID(address, newID);
              dxl1.torqueOn(newID);
            break;
            case 2:
              dxl2.torqueOff(address);
              result = dxl2.setID(address, newID);
              dxl2.torqueOn(newID);
            break;
            case 3:
              dxl3.torqueOff(address);
              result = dxl3.setID(address, newID);
              dxl3.torqueOn(newID);
            break;
          }
          USBCOM.writeByte(result);
        }
      break;
    }
  }
  // Update tracked movements
  if (micros() - monitorStartTime > 1000) {
    for (int chan = 0; chan < 3; chan++) {
      for (int addr = 0; addr < MAX_MOTOR_ADDR; addr++) {
        if (trackMovement[chan][addr]) {
          switch(chan+1) {
            case 1:
              pos = dxl1.getPresentPosition(addr+1, UNIT_DEGREE);
            break;
            case 2:
              pos = dxl2.getPresentPosition(addr+1, UNIT_DEGREE);
            break;
            case 3:
              pos = dxl3.getPresentPosition(addr+1, UNIT_DEGREE);
            break;
          }
          thisMovementGoal = trackMovementGoal[chan][addr];
          if (abs(pos-thisMovementGoal) < 1) {
            if (trackMovementSource[chan][addr] == 1) {
              StateMachineCOM.writeByte(eventCode[chan][addr][1]);
            }
            trackMovement[chan][addr] = false;
            if (motorMode[chan][addr] == 5) { // In step mode, clear position
              delayMicroseconds(100000); // Wait for movement to fully end (Todo: Replace with better indicator!)
              resetExtendedPosition(chan+1, addr+1);
              if (thisMovementGoal > 0) {
                  thisMovementGoal = thisMovementGoal-(floor(thisMovementGoal/360)*360);
              } else if (thisMovementGoal < 0) {
                  thisMovementGoal = 360-(abs(thisMovementGoal)-(floor(abs(thisMovementGoal)/360)*360));
              }
              currentPosition[chan][addr] = thisMovementGoal;
            }
          }
        }
      }
    }
  }
}

void programHandler() {
  uint8_t thisStep = 0;
  uint8_t thisAddress = 0;
  uint8_t thisChannel = 0;

  // Update DIO
  for (int i = 0; i < 3; i++) {
    if (!inDebounce[i]) {
      currentDioState[i] = digitalReadFast(dioPins[i]);
      if (currentDioState[i] == 0 && lastDioState[i] == 1) {
        switch(dio_FallingEdgeFcn[i]) {
          case 1:
            startProgram(dio_TargetProgram[i], 2);
          break;
          case 2:
            program_running[dio_TargetProgram[i]] = false;
          break;
          case 3:
            emergency_stop();
          break;
        }
        inDebounce[i] = true;
      } else if (currentDioState[i] == 1 && lastDioState[i] == 0) {
        switch(dio_RisingEdgeFcn[i]) {
          case 1:
            startProgram(dio_TargetProgram[i], 2);
          break;
          case 2:
            stopProgram(dio_TargetProgram[i]);
          break;
          case 3:
            emergency_stop();
          break;
        }
        inDebounce[i] = true;
      }
      lastDioState[i] = currentDioState[i];
    } else { // Debounce period
      debounceCounter[i]++;
      if (debounceCounter[i] > dio_Debounce[i]) {
        debounceCounter[i] = 0;
        inDebounce[i] = false;
      }
    }
  }

  // Run motor program step(s)
  for (int prog = 0; prog < MAX_PROGRAMS; prog++) {
    if (program_running[prog]) {
      thisStep = program_currentStep[prog];
      while (program_stepTime[prog][thisStep] == program_currentTime[prog] && thisStep < program_nSteps[prog]) {
        thisChannel = program_channel[prog][thisStep];
        thisAddress = program_address[prog][thisStep];
        if (program_moveType[prog] == 0) {
          writeMaxVelocity_Fast(thisChannel, thisAddress, program_goalMovementLimit[prog][thisStep]);
        } else {
          writeMaxCurrent_Fast(thisChannel, thisAddress, program_goalMovementLimit[prog][thisStep]);
        }
        writeGoalPosition_Fast(thisChannel, thisAddress, program_goalPosition[prog][thisStep]);

        if (thisChannel < 4 && thisAddress < 4 && program_origin[prog] == 1) {
          StateMachineCOM.writeByte(eventCode[thisChannel-1][thisAddress-1][0]);
        }
        thisStep++;
        program_currentStep[prog] = thisStep;
      }
      program_currentTime[prog]++;
      if (program_currentStep[prog] == program_nSteps[prog]) {
        program_currentStep[prog] = 0;
        program_currentTime[prog] = 0;
        if (program_nLoops[prog] == 0) {
          program_running[prog] = false;
        } else {
          program_loopCount[prog]++;
          if (program_loopCount[prog] >= program_nLoops[prog]) {
            program_running[prog] = false;
          }
        } 
      }
    }
  }
}

void setMotorMode(uint8_t channel, uint8_t address, uint8_t modeIndex) {
  // Convert mode index to dynamixel mode index
  uint8_t dynamixelModeIndex = 0;
  switch(modeIndex) {
    case 1: // Position mode
      dynamixelModeIndex = 3;
    break;
    case 2: // Extended Position Mode
      dynamixelModeIndex = 4;
    break;
    case 3: // Current-Limited Position Mode
      dynamixelModeIndex = 5;
    break;
    case 4: // Velocity Mode
      dynamixelModeIndex = 1;
    break;
    case 5: // Step Mode (Extended position + auto position reset after each move)
      dynamixelModeIndex = 4;
    break;
  }
  switch(channel) {
    case 1:
      dxl1.torqueOff(address);
      dxl1.setOperatingMode(address, dynamixelModeIndex);
      dxl1.torqueOn(address);
    break;
    case 2:
      dxl2.torqueOff(address);
      dxl2.setOperatingMode(address, dynamixelModeIndex);
      dxl2.torqueOn(address);
    break;
    case 3:
      dxl3.torqueOff(address);
      dxl3.setOperatingMode(address, dynamixelModeIndex);
      dxl3.torqueOn(address);
    break;
  }
  motorMode[channel-1][address-1] = modeIndex;
}

void setGoalPosition(uint8_t channel, uint8_t address, float newPosition) {
  if ((motorMode[channel-1][address-1] == 1) || (motorMode[channel-1][address-1] == 2) 
                                         || (motorMode[channel-1][address-1] == 3)
                                         || (motorMode[channel-1][address-1] == 5)) {
    writeGoalPosition_Fast(channel, address, newPosition);
    trackMovementSource[channel-1][address-1] = opSource;
    trackMovementGoal[channel-1][address-1] = newPosition;
    trackMovement[channel-1][address-1] = true;
    if (opSource == 0) {
      USBCOM.writeByte(1);
    } else {
      StateMachineCOM.writeByte(eventCode[channel-1][address-1][0]);
    }
  } else {
    if (opSource == 0) {
      USBCOM.writeByte(0);
    }
  }
}

void writeGoalPosition_Fast(uint8_t chan, uint8_t addr, float newPosition) {
  // Sets the goal position
  // Args:
  // chan: Channel (1-3)
  // addr: Address within channel (1-3)
  // newPosition: The new goal position. Units = degrees
  int32_t newPos_Tics = 0;
  newPos_Tics = (int32_t)round(newPosition*11.375);
  typeBuffer.int32 = newPos_Tics;
  switch (chan) {
    case 1:
      dxl1.writeNoResp(addr, 116, typeBuffer.byteArray, 4);
    break;
    case 2:
      dxl2.writeNoResp(addr, 116, typeBuffer.byteArray, 4);
    break;
    case 3:
      dxl3.writeNoResp(addr, 116, typeBuffer.byteArray, 4);
    break;
  }
}

void writeGoalVelocity_Fast(uint8_t chan, uint8_t addr, float newVelocity) {
  // Sets the goal velocity (only used in velocity control mode, aka Speed mode)
  // Args:
  // chan: Channel (1-3)
  // addr: Address within channel (1-3)
  // newPosition: The new goal position. Units = degrees
  int32_t newVel_Units = 0;
  newVel_Units = (int32_t)round(newVelocity*vel2Units);
  typeBuffer.int32 = newVel_Units;
  switch (chan) {
    case 1:
      dxl1.writeNoResp(addr, 104, typeBuffer.byteArray, 4);
    break;
    case 2:
      dxl2.writeNoResp(addr, 104, typeBuffer.byteArray, 4);
    break;
    case 3:
      dxl3.writeNoResp(addr, 104, typeBuffer.byteArray, 4);
    break;
  }
}

void writeMaxVelocity_Fast(uint8_t chan, uint8_t addr, float newVelocity) {
  // Sets the maximum velocity for subsequent position moves
  // Args:
  // chan: Channel (1-3)
  // addr: Address within channel (1-3)
  // newVelocity: The maximum velocity for subsequent moves. Units = rev/s
  int32_t newVel_Units = 0;
  newVel_Units = (int32_t)round(newVelocity*vel2Units); // Convert from rev per second to units
  typeBuffer.int32 = newVel_Units;
  switch (chan) {
    case 1:
      dxl1.writeNoResp(addr, 112, typeBuffer.byteArray, 4);
      //dxl1.writeControlTableItem(PROFILE_VELOCITY, addr, newVelocity);
    break;
    case 2:
      dxl2.writeNoResp(addr, 112, typeBuffer.byteArray, 4);
    break;
    case 3:
      dxl3.writeNoResp(addr, 112, typeBuffer.byteArray, 4);
    break;
  }
}

void writeMaxCurrent_Fast(uint8_t chan, uint8_t addr, float newMaxCurrent) {
  int16_t newCurrent_Units = 0;
  newCurrent_Units = (int16_t)round(newMaxCurrent);
  typeBuffer.int16 = newCurrent_Units;
  switch (chan) {
    case 1:
      dxl1.writeNoResp(addr, 102, typeBuffer.byteArray, 2);
    break;
    case 2:
      dxl2.writeNoResp(addr, 102, typeBuffer.byteArray, 2);
    break;
    case 3:
      dxl3.writeNoResp(addr, 102, typeBuffer.byteArray, 2);
    break;
  }
}

void writeMaxAcceleration_Fast(uint8_t chan, uint8_t addr, float newAcceleration) {
  int32_t newAcc_Units = 0;
  newAcc_Units = (int32_t)round(newAcceleration*acc2Units); // Convert from rev per second^2 to units
  typeBuffer.int32 = newAcc_Units;
  switch (chan) {
    case 1:
      dxl1.writeNoResp(addr, 108, typeBuffer.byteArray, 4);
    break;
    case 2:
      dxl2.writeNoResp(addr, 108, typeBuffer.byteArray, 4);
    break;
    case 3:
      dxl3.writeNoResp(addr, 108, typeBuffer.byteArray, 4);
    break;
  }
}

void stepMotor(uint8_t channel, uint8_t address, float newPosition) {
  setGoalPosition(channel, address, newPosition);
}

void delayUntilMovementEnd(uint8_t channel, uint8_t address, float targetPosition) {
  float pos = 0;
  switch (channel) {
    case 1:
      pos = dxl1.getPresentPosition(address, UNIT_DEGREE);
      while (abs(pos-targetPosition) > 1) {
        pos = dxl1.getPresentPosition(address, UNIT_DEGREE);
      }
    break;
    case 2:
      pos = dxl2.getPresentPosition(address, UNIT_DEGREE);
      while (abs(pos-targetPosition) > 1) {
        pos = dxl2.getPresentPosition(address, UNIT_DEGREE);
      }
    break;
    case 3:
      pos = dxl3.getPresentPosition(address, UNIT_DEGREE);
      while (abs(pos-targetPosition) > 1) {
        pos = dxl3.getPresentPosition(address, UNIT_DEGREE);
      }
    break;
  }
  delay(100); // A safe margin for the final degree
}

void resetExtendedPosition(uint8_t channel, uint8_t address) {
  switch (channel) {
    case 1:
      dxl1.torqueOff(address);
      dxl1.clear(address, 0x01, 0, 1000);
      dxl1.torqueOn(address);
    break;
    case 2:
      dxl2.torqueOff(address);
      dxl2.clear(address, 0x01, 0, 1000);
      dxl2.torqueOn(address);
    break;
    case 3:
      dxl3.torqueOff(address);
      dxl3.clear(address, 0x01, 0, 1000);
      dxl3.torqueOn(address);
    break;
  }
}

void startProgram(uint8_t programID, uint8_t opSource) {
  program_currentStep[programID] = 0;
  program_currentTime[programID] = 0;
  program_loopCount[programID] = 0;
  program_origin[programID] = opSource;
  program_running[programID] = true;
}

void stopProgram(uint8_t programID) {
  program_running[programID] = false;
}

void emergency_stop() { // Stop all motors, leaving torque off. Motors must then be individually reinitialized by setting motor mode.
  for (int i = 0; i < 3; i++) {
    program_running[i] = false;
  }
  for (int addr = 0; addr < MAX_MOTOR_ADDR; addr++) {
    if (motorDetected[0][addr]) {
      dxl1.torqueOff(addr);
    }
    if (motorDetected[1][addr]) {
      dxl2.torqueOff(addr);
    }
    if (motorDetected[2][addr]) {
      dxl3.torqueOff(addr);
    }
  }
}

void clearMotorPrograms() {
  for (int i = 0; i < MAX_PROGRAMS; i++) {
    program_loaded[programID] = false;
  }
}

void discoverMotors(bool useUSB) {
  int i = 0;
  bool found = false;
  // Detect on channel 1
  for (int motorIndex = 0; motorIndex < MAX_MOTOR_ADDR; motorIndex++) {
    found = false;
    i = 0;
    while ((found == false) && (i < 7)) {
      dxl1.begin(validBaudRates[i]);
      if (dxl1.ping(motorIndex)) {
        found = true;
        motorDetected[0][motorIndex] = true;
        dxl1.torqueOff(motorIndex);
        dxl1.setBaudrate(motorIndex, 4000000);
        dxl1.begin(4000000);
        dxl1.torqueOn(motorIndex);
        if (useUSB) {
          USBCOM.writeByte(1);
          USBCOM.writeByte(motorIndex);
          USBCOM.writeUint32(dxl1.getModelNumber(motorIndex));
        }
      }
      delay(1);
      i++;
    }
  }
    // Detect on channel 2
  for (int motorIndex = 0; motorIndex < MAX_MOTOR_ADDR; motorIndex++) {
    found = false;
    i = 0;
    while ((found == false) && (i < 7)) {
      dxl2.begin(validBaudRates[i]);
      if (dxl2.ping(motorIndex)) {
        found = true;
        motorDetected[1][motorIndex] = true;
        dxl2.torqueOff(motorIndex);
        dxl2.setBaudrate(motorIndex, 4000000);
        dxl2.begin(4000000);
        dxl2.torqueOn(motorIndex);
        if (useUSB) {
          USBCOM.writeByte(2);
          USBCOM.writeByte(motorIndex);
          USBCOM.writeUint32(dxl2.getModelNumber(motorIndex));
        }
      }
      delay(1);
      i++;
    }
  }
    // Detect on channel 3
  for (int motorIndex = 0; motorIndex < MAX_MOTOR_ADDR; motorIndex++) {
    found = false;
    i = 0;
    while ((found == false) && (i < 7)) {
      dxl3.begin(validBaudRates[i]);
      if (dxl3.ping(motorIndex)) {
        found = true;
        motorDetected[2][motorIndex] = true;
        dxl3.torqueOff(motorIndex);
        dxl3.setBaudrate(motorIndex, 4000000);
        dxl3.begin(4000000);
        dxl3.torqueOn(motorIndex);
        if (useUSB) {
          USBCOM.writeByte(3);
          USBCOM.writeByte(motorIndex);
          USBCOM.writeUint32(dxl3.getModelNumber(motorIndex));
        }
      }
      delay(1);
      i++;
    }
  }
  dxl1.begin(4000000);
  dxl2.begin(4000000);
  dxl3.begin(4000000);
}

byte readByteFromSource(byte opSource) {
  switch (opSource) {
    case 0:
      return USBCOM.readByte();
    break;
    case 1:
      return StateMachineCOM.readByte();
    break;
  }
}

float readFloatFromSource(byte opSource) {
  switch (opSource) {
    case 0:
      return USBCOM.readFloat();
    break;
    case 1:
      return StateMachineCOM.readFloat();
    break;
  }
}

void returnModuleInfo() {
  boolean fsmSupportsHwInfo = false;
  delayMicroseconds(100);
  if (StateMachineCOM.available() == 1) { // FSM firmware v23 or newer sends a second info request byte to indicate that it supports additional ops
    if (StateMachineCOM.readByte() == 255) {fsmSupportsHwInfo = true;}
  }
  StateMachineCOM.writeByte(65); // Acknowledge
  StateMachineCOM.writeUint32(FIRMWARE_VERSION); // 4-byte firmware version
  StateMachineCOM.writeByte(sizeof(moduleName)-1); // Length of module name
  StateMachineCOM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
  StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
  StateMachineCOM.writeByte('#'); // Op code for: Number of behavior events this module can generate
  StateMachineCOM.writeByte(18); // 3 channels x 3 motors
  StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
  StateMachineCOM.writeByte('E'); // Op code for: Behavior event names
  StateMachineCOM.writeByte(nEventNames);
  for (int i = 0; i < nEventNames; i++) { // Once for each event name
    StateMachineCOM.writeByte(strlen(eventNames[i])); // Send event name length
    for (int j = 0; j < strlen(eventNames[i]); j++) { // Once for each character in this event name
      StateMachineCOM.writeByte(*(eventNames[i]+j)); // Send the character
    }
  }
  if (fsmSupportsHwInfo) {
    StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('V'); // Op code for: Hardware major version
    StateMachineCOM.writeByte(HARDWARE_VERSION); 
    StateMachineCOM.writeByte(1); // 1 if more info follows, 0 if not
    StateMachineCOM.writeByte('v'); // Op code for: Hardware minor version
    StateMachineCOM.writeByte(circuitRevision); 
  }
  StateMachineCOM.writeByte(0); // 1 if more info follows, 0 if not
}
