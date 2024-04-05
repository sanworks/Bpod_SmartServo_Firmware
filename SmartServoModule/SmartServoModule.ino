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

// ---Macros---
// Pins
#define DXL_DIR_PIN1 12
#define DXL_DIR_PIN2 18
#define DXL_DIR_PIN3 19

// HW + FW versions
#define FIRMWARE_VERSION 1
#define HARDWARE_VERSION 1

// Motor programs
#define MAX_STEPS 100
#define MAX_PROGRAMS 256

// Module properties
char moduleName[] = "SmartServo"; 
const byte dioPins[3] = {9,8,7};
const byte circuitRevisionArray[5] = {2,3,4,5,6};

// Namespace (for Dynamixel control table items)
using namespace ControlTableItem;  

// Setup interfaces
ArCOM USBCOM(Serial);
ArCOM StateMachineCOM(Serial1);
byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer
Dynamixel2Arduino dxl1(Serial3, DXL_DIR_PIN1);
Dynamixel2Arduino dxl2(Serial4, DXL_DIR_PIN2);
Dynamixel2Arduino dxl3(Serial5, DXL_DIR_PIN3);

// Hardware timer
IntervalTimer hardwareTimer; // Hardware timer to create even sampling

// Constants
const uint32_t validBaudRates[7] = {4000000, 57600, 1000000, 2000000, 3000000, 115200, 9600}; // Ordered by most likely encounters during auto-detection

// Motor program metadata
bool program_Loaded[MAX_PROGRAMS] = {false}; // true for each program if a motor program was loaded via USB
bool program_Running[MAX_PROGRAMS] = {false}; // true for each program if the program was triggered and is actively executing
uint8_t program_nSteps[MAX_PROGRAMS] = {false}; // Number of steps in each program
uint8_t program_currentStep[MAX_PROGRAMS] = {0}; // Tracks the current step in each program as it is executed
uint32_t program_currentTime[MAX_PROGRAMS] = {0}; // Tracks the current time with respect to program start as each program is executed
uint32_t program_currentLoopTime[MAX_PROGRAMS] = {0}; // Tracks the current time with respect to trigger start as each looping program is executed

// Motor program content
uint8_t program_Channel[MAX_PROGRAMS][MAX_STEPS] = {0}; // Motor channel for each instruction in each motor program
uint8_t program_Address[MAX_PROGRAMS][MAX_STEPS] = {0}; // Motor address for each instruction in each motor program
float program_GoalPosition[MAX_PROGRAMS][MAX_STEPS] = {0}; // Goal position for each instruction in each motor program
float program_GoalVelocity[MAX_PROGRAMS][MAX_STEPS] = {0}; // Goal velocity for each instruction in each motor program
float program_GoalAcceleration[MAX_PROGRAMS][MAX_STEPS] = {0}; // Goal acceleration for each instruction in each motor program
uint32_t program_StepTime[MAX_PROGRAMS][MAX_STEPS] = {0}; // The time to execute each step in each motor program, with respect to program start
uint32_t program_LoopTime[MAX_PROGRAMS] = {0}; // If >0, the program loops for a given amount of time

// General variables
uint8_t motorMode[3][8] = {0}; // 1 = Position, 2 = Extended Position, 3 = Current-Limited Position, 4 = Velocity
uint8_t address = 1; // Motor address on a channel (1-253)
uint8_t channel = 1; // Hardware serial channel targeted (1-3)
uint8_t newMode = 1; // Motor mode
uint8_t newID = 0; // New motor address
uint8_t programID = 0; // ID of the current program
uint8_t tableIndex = 0; // Control table index
uint8_t opCode = 0; // Op code, a byte code to identify each operation
uint8_t opSource = 0; // Op source, 0 = USB, 1 = State Machine
uint8_t focusChannel = 0; // The channel currently in focus
uint8_t focusAddress = 0; // The motor address currently in focus
uint8_t blocking = 0; // 0: confirm new position goal immediately 1: confirm after movement finishes executing
uint32_t nSteps = 0; 
float value = 0; // Temporary float
float maxVelocity = 0; // Maximum velocity
float maxAccel = 0; // Maximum acceleration
float pos = 0; // Position (degrees)
float current = 0; // Current limit (percent)
float newVelocity = 0; // Velocity (RPM)
uint8_t circuitRevision = 0;


void setup() {
  // Setup motor channels
  dxl1.begin(4000000);
  dxl1.setPortProtocolVersion(2.0);
  dxl2.begin(4000000);
  dxl2.setPortProtocolVersion(2.0);
  dxl3.begin(4000000);
  dxl3.setPortProtocolVersion(2.0);

  // Setup state machine UART serial port
  Serial1.begin(1312500);
  Serial1.addMemoryForRead(StateMachineSerialBuf, 192);

  // Initialize DIO pins as input (high impedance)
  for (int i = 0; i < 3; i++) {
    pinMode(dioPins[i], INPUT);
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

  // Start hardware timer
  hardwareTimer.begin(programHandler, 100);
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
        switch(channel) {
            case 1:
              dxl1.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
            break;
            case 2:
              dxl2.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
            break;
            case 3:
              dxl3.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
            break;
          }
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
      break;

      case ']': // Set max acceleration
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        maxAccel = readFloatFromSource(opSource);
        switch(channel) {
            case 1:
              dxl1.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
            break;
            case 2:
              dxl2.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
            break;
            case 3:
              dxl3.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
            break;
          }
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
        if (motorMode[channel][address] == 1) {
          switch(channel) {
            case 1:
              dxl1.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
              dxl1.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
              dxl1.setGoalPosition(address, pos, UNIT_DEGREE);
            break;
            case 2:
              dxl2.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
              dxl2.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
              dxl2.setGoalPosition(address, pos, UNIT_DEGREE);
            break;
            case 3:
              dxl3.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
              dxl3.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
              dxl3.setGoalPosition(address, pos, UNIT_DEGREE);
            break;
          }
          if (blocking) {
            delayUntilMovementEnd(channel, address, pos);
          }
          if (opSource == 0) {
            USBCOM.writeByte(1);
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
        if (motorMode[channel][address] == 3) {
          switch(channel) {
            case 1:
              dxl1.setGoalCurrent(address, current, UNIT_PERCENT);
              dxl1.setGoalPosition(address, pos, UNIT_DEGREE);
            break;
            case 2:
              dxl2.setGoalCurrent(address, current, UNIT_PERCENT);
              dxl2.setGoalPosition(address, pos, UNIT_DEGREE);
            break;
            case 3:
              dxl3.setGoalCurrent(address, current, UNIT_PERCENT);
              dxl3.setGoalPosition(address, pos, UNIT_DEGREE);
            break;
          }
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
        } else {
          if (opSource == 0) {
            USBCOM.writeByte(0);
          }
        }
      break;

      case 'V': // Set velocity goal (in motor mode 4). Units = RPM. Sign indicates direction.
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        newVelocity = readFloatFromSource(opSource);
        if (motorMode[channel][address] == 4) {
          switch(channel) {
            case 1:
              dxl1.setGoalVelocity(address, newVelocity, UNIT_RPM);
            break;
            case 2:
              dxl2.setGoalVelocity(address, newVelocity, UNIT_RPM);
            break;
            case 3:
              dxl3.setGoalVelocity(address, newVelocity, UNIT_RPM);
            break;
          }
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
        } else {
          if (opSource == 0) {
            USBCOM.writeByte(0);
          }
        }
      break;

      case 'L': // Load a motor program
        if (opSource == 0) {
          programID = USBCOM.readByte();
          nSteps = USBCOM.readByte();
          program_nSteps[programID] = nSteps;
          program_LoopTime[programID] = USBCOM.readUint32();
          for (int i = 0; i < nSteps; i++) {
            program_Channel[programID][i] = USBCOM.readByte();
          }
          for (int i = 0; i < nSteps; i++) {
            program_Address[programID][i] = USBCOM.readByte();
          }
          for (int i = 0; i < nSteps; i++) {
            program_GoalPosition[programID][i] = USBCOM.readFloat();
          }
          for (int i = 0; i < nSteps; i++) {
            program_GoalVelocity[programID][i] = USBCOM.readFloat();
          }
          for (int i = 0; i < nSteps; i++) {
            program_GoalAcceleration[programID][i] = USBCOM.readFloat();
          }
          for (int i = 0; i < nSteps; i++) {
            program_StepTime[programID][i] = USBCOM.readUint32();
          }
          program_Loaded[programID] = true;
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case 'R': // Run motor program
        programID = readByteFromSource(opSource);
        program_currentStep[programID] = 0;
        program_currentTime[programID] = 0;
        program_currentLoopTime[programID] = 0;
        program_Running[programID] = true;
      break;

      case '+': // Return current position via USB
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
              dxl1.setID(address, newID);
            break;
            case 2:
              dxl2.setID(address, newID);
            break;
            case 3:
              dxl3.setID(address, newID);
            break;
          }
          USBCOM.writeByte(1);
        }
      break;
    }
  }
}

void programHandler() {
  uint8_t thisStep = 0;
  uint8_t thisAddress = 0;
  for (int prog = 0; prog < MAX_PROGRAMS; prog++) {
    if (program_Running[prog]) {
      thisStep = program_currentStep[prog];
      while (program_StepTime[prog][thisStep] == program_currentTime[prog]) {
        thisAddress = program_Address[prog][thisStep];
        switch(program_Channel[prog][thisStep]) {
          case 1:
            dxl1.writeControlTableItem(PROFILE_VELOCITY, thisAddress, program_GoalVelocity[prog][thisStep]);
            dxl1.writeControlTableItem(PROFILE_ACCELERATION, thisAddress, program_GoalAcceleration[prog][thisStep]);
            dxl1.setGoalPosition(thisAddress, program_GoalPosition[prog][thisStep], UNIT_DEGREE);
          break;
          case 2:
            dxl2.writeControlTableItem(PROFILE_VELOCITY, thisAddress, program_GoalVelocity[prog][thisStep]);
            dxl2.writeControlTableItem(PROFILE_ACCELERATION, thisAddress, program_GoalAcceleration[prog][thisStep]);
            dxl2.setGoalPosition(thisAddress, program_GoalPosition[prog][thisStep], UNIT_DEGREE);
          break;
          case 3:
            dxl3.writeControlTableItem(PROFILE_VELOCITY, thisAddress, program_GoalVelocity[prog][thisStep]);
            dxl3.writeControlTableItem(PROFILE_ACCELERATION, thisAddress, program_GoalAcceleration[prog][thisStep]);
            dxl3.setGoalPosition(thisAddress, program_GoalPosition[prog][thisStep], UNIT_DEGREE);
          break;
        }
        thisStep++;
        program_currentStep[prog] = thisStep;
      }
      program_currentTime[prog]++;
      if (program_currentStep[prog] == program_nSteps[prog]) {
        program_currentStep[prog] = 0;
        program_currentTime[prog] = 0;
        if (program_LoopTime[prog] == 0) {
          program_Running[prog] = false;
        }  
      }
      if (program_LoopTime[prog] > 0) {
        program_currentLoopTime[prog]++;
        if (program_currentLoopTime[prog] >= program_LoopTime[prog]) {
          program_Running[prog] = false;
        }
      }
    }
  }
}



void setMotorMode(uint8_t channel, uint8_t motorIndex, uint8_t modeIndex) {
  // Convert mode index to dynamixel mode index
  uint8_t dynamixelModeIndex = 0;
  switch(modeIndex) {
    case 1:
      dynamixelModeIndex = 3;
    break;
    case 2:
      dynamixelModeIndex = 4;
    break;
    case 3:
      dynamixelModeIndex = 5;
    break;
    case 4:
      dynamixelModeIndex = 1;
    break;
  }
  switch(channel) {
    case 1:
      dxl1.torqueOff(motorIndex);
      dxl1.setOperatingMode(motorIndex, dynamixelModeIndex);
      dxl1.torqueOn(motorIndex);
    break;
    case 2:
      dxl2.torqueOff(motorIndex);
      dxl2.setOperatingMode(motorIndex, dynamixelModeIndex);
      dxl2.torqueOn(motorIndex);
    break;
    case 3:
      dxl3.torqueOff(motorIndex);
      dxl3.setOperatingMode(motorIndex, dynamixelModeIndex);
      dxl3.torqueOn(motorIndex);
    break;
  }
  motorMode[channel][motorIndex] = modeIndex;
}

void setGoalPosition(uint8_t channel, uint8_t address, float newPosition) {
  if ((motorMode[channel][address] == 1) || (motorMode[channel][address] == 2)) {
    switch(channel) {
      case 1:
        dxl1.setGoalPosition(address, newPosition, UNIT_DEGREE);
      break;
      case 2:
        dxl2.setGoalPosition(address, newPosition, UNIT_DEGREE);
      break;
      case 3:
        dxl3.setGoalPosition(address, newPosition, UNIT_DEGREE);
      break;
    }
    if (opSource == 0) {
      USBCOM.writeByte(1);
    }
  } else {
    if (opSource == 0) {
      USBCOM.writeByte(0);
    }
  }
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
}

void clearMotorPrograms() {
  for (int i = 0; i < MAX_PROGRAMS; i++) {
    program_Loaded[programID] = false;
  }
}

void discoverMotors(bool useUSB) {
  int i = 0;
  bool found = false;
  // Detect on channel 1
  for (int motorIndex = 0; motorIndex < 16; motorIndex++) {
    found = false;
    i = 0;
    while ((found == false) && (i < 7)) {
      dxl1.begin(validBaudRates[i]);
      if (dxl1.ping(motorIndex)) {
        found = true;
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
      i++;
    }
  }
    // Detect on channel 2
  for (int motorIndex = 0; motorIndex < 16; motorIndex++) {
    found = false;
    i = 0;
    while ((found == false) && (i < 7)) {
      dxl2.begin(validBaudRates[i]);
      if (dxl2.ping(motorIndex)) {
        found = true;
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
      i++;
    }
  }
    // Detect on channel 3
  for (int motorIndex = 0; motorIndex < 16; motorIndex++) {
    found = false;
    i = 0;
    while ((found == false) && (i < 7)) {
      dxl3.begin(validBaudRates[i]);
      if (dxl3.ping(motorIndex)) {
        found = true;
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
