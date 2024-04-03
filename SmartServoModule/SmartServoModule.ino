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

// Name of module for manual override UI and state machine assembler
char moduleName[] = "SmartServo"; 
uint8_t circuitRevision = 0;

// Namespace (for Dynamixel control table items)
using namespace ControlTableItem;  

// Setup interfaces
ArCOM USBCOM(Serial);
ArCOM StateMachineCOM(Serial1);
byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer
Dynamixel2Arduino dxl1(Serial3, DXL_DIR_PIN1);
Dynamixel2Arduino dxl2(Serial4, DXL_DIR_PIN2);
Dynamixel2Arduino dxl3(Serial5, DXL_DIR_PIN3);

// Constants
const uint32_t validBaudRates[7] = {4000000, 57600, 1000000, 2000000, 3000000, 9600, 115200}; // Ordered by most likely encounters during auto-detection

// Program variables
uint8_t motorMode[3][8] = {0}; // 1 = Position, 2 = Extended Position, 3 = Current-Limited Position, 4 = Velocity
uint8_t address = 1; // Motor address on a channel (1-253)
uint8_t channel = 1; // Hardware serial channel targeted (1-3)
uint8_t newMode = 1; // Motor mode
uint8_t newID = 0; // New motor address
uint8_t tableIndex = 0; // Control table index
uint8_t opCode = 0; // Op code, a byte code to identify each operation
uint8_t opSource = 0; // Op source, 0 = USB, 1 = State Machine
uint8_t focusChannel = 0; // The channel currently in focus
uint8_t focusAddress = 0; // The motor address currently in focus
float value = 0; // Temporary float
float maxVelocity = 0; // Maximum velocity
float maxAccel = 0; // Maximum acceleration
float pos = 0; // Position (degrees)
float current = 0; // Current limit (percent)
float newVelocity = 0; // Velocity (RPM)


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
}

void loop() {
  opCode = 0;
  if (StateMachineCOM.available() > 0) {
      opCode = StateMachineCOM.readByte();
      opSource = 1;
  } else if (USBCOM.available() > 0) {
      opCode = USBCOM.readByte();
      opSource = 0;
  }
  if (opCode > 0) {
    switch(opCode) {
      case 255: // Return Bpod module info
        if (opSource == 1) { // Only returns this info if requested from state machine device
          returnModuleInfo();
        }
      break;
      case 'D': // Discover motors, initialize and return metadata
        discoverMotors(true); // Arg: true returns metadata via USB, false does not
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

      case 'F': // Set channel and address of motor in focus, for focused movement commands
        focusChannel = readByteFromSource(opSource);
        focusAddress = readByteFromSource(opSource);
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

      case 'P': // Set position goal of a target motor
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        setGoalPosition(channel, address, pos);
      break;

      case 'p': // Set position of the motor in focus
        pos = readFloatFromSource(opSource);
        if (focusChannel > 0 && focusAddress  > 0) {
          setGoalPosition(focusChannel, focusAddress, pos);
        }
      break;

      case 'G': // Set position goal with a new velocity and acceleration
        channel = readByteFromSource(opSource);
        address = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        maxVelocity = readFloatFromSource(opSource);
        maxAccel = readFloatFromSource(opSource);
        if (motorMode[channel][address] == 1) {
          switch(channel) {
            case 1:
              dxl1.torqueOff(address);
              dxl1.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
              dxl1.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
              dxl1.torqueOn(address);
              dxl1.setGoalPosition(address, pos, UNIT_DEGREE);
              
            break;
            case 2:
              dxl2.torqueOff(address);
              dxl2.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
              dxl2.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
              dxl2.torqueOn(address);
              dxl2.setGoalPosition(address, pos, UNIT_DEGREE);
            break;
            case 3:
              dxl3.torqueOff(address);
              dxl3.writeControlTableItem(PROFILE_VELOCITY, address, maxVelocity);
              dxl3.writeControlTableItem(PROFILE_ACCELERATION, address, maxAccel);
              dxl3.torqueOn(address);
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

      case 'V': // Set velocity goal (in motor mode 4). Sign indicates direction.
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

      case 'R': // Return current position via USB
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
              USBCOM.writeFloat(dxl1.readControlTableItem(tableIndex, address));
            break;
            case 2:
              USBCOM.writeFloat(dxl2.readControlTableItem(tableIndex, address));
            break;
            case 3:
              USBCOM.writeFloat(dxl3.readControlTableItem(tableIndex, address));
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

void setGoalPosition(byte channel, byte address, float newPosition) {
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
