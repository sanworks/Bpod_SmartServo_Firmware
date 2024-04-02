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

// Program variables
uint8_t motorInitialized[3][64] = {0}; // 1 = Initialized, 0 = Not
uint8_t motorMode[3][64] = {0}; // 1 = Position, 2 = Extended Position, 3 = Current-Limited Position, 4 = Velocity
uint8_t motorID = 1; // Motor ID
uint8_t channel = 1; // Hardware serial channel targeted
uint8_t newMode = 1; // Motor mode
uint8_t opCode = 0; // Op code, a byte code to identify each operation
uint8_t opSource = 0; // Op source, 0 = USB, 1 = State Machine
float value = 0;
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

  // Initialize motor 1 on each channel
  startup_motor(1, 1);
  startup_motor(2, 1);
  startup_motor(3, 1);

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
      case '[': // Set max velocity
        channel = readByteFromSource(opSource);
        motorID = readByteFromSource(opSource);
        maxVelocity = readFloatFromSource(opSource);
        switch(channel) {
            case 1:
              dxl1.writeControlTableItem(PROFILE_VELOCITY, motorID, maxVelocity);
            break;
            case 2:
              dxl2.writeControlTableItem(PROFILE_VELOCITY, motorID, maxVelocity);
            break;
            case 3:
              dxl3.writeControlTableItem(PROFILE_VELOCITY, motorID, maxVelocity);
            break;
          }
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
      break;

      case ']': // Set max acceleration
        channel = readByteFromSource(opSource);
        motorID = readByteFromSource(opSource);
        maxAccel = readFloatFromSource(opSource);
        switch(channel) {
            case 1:
              dxl1.writeControlTableItem(PROFILE_ACCELERATION, motorID, maxAccel);
            break;
            case 2:
              dxl2.writeControlTableItem(PROFILE_ACCELERATION, motorID, maxAccel);
            break;
            case 3:
              dxl3.writeControlTableItem(PROFILE_ACCELERATION, motorID, maxAccel);
            break;
          }
          if (opSource == 0) {
            USBCOM.writeByte(1);
          }
      break;

      case 'P': // Set position goal
        channel = readByteFromSource(opSource);
        motorID = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        if ((motorMode[channel][motorID] == 1) || (motorMode[channel][motorID] == 2)) {
          switch(channel) {
            case 1:
              dxl1.setGoalPosition(motorID, pos, UNIT_DEGREE);
            break;
            case 2:
              dxl2.setGoalPosition(motorID, pos, UNIT_DEGREE);
            break;
            case 3:
              dxl3.setGoalPosition(motorID, pos, UNIT_DEGREE);
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

      case 'G': // Set position goal with a new velocity and acceleration
        channel = readByteFromSource(opSource);
        motorID = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        maxVelocity = readFloatFromSource(opSource);
        maxAccel = readFloatFromSource(opSource);
        if (motorMode[channel][motorID] == 1) {
          switch(channel) {
            case 1:
              dxl1.torqueOff(motorID);
              dxl1.writeControlTableItem(PROFILE_VELOCITY, motorID, maxVelocity);
              dxl1.writeControlTableItem(PROFILE_ACCELERATION, motorID, maxAccel);
              dxl1.torqueOn(motorID);
              dxl1.setGoalPosition(motorID, pos, UNIT_DEGREE);
              
            break;
            case 2:
              dxl2.torqueOff(motorID);
              dxl2.writeControlTableItem(PROFILE_VELOCITY, motorID, maxVelocity);
              dxl2.writeControlTableItem(PROFILE_ACCELERATION, motorID, maxAccel);
              dxl2.torqueOn(motorID);
              dxl2.setGoalPosition(motorID, pos, UNIT_DEGREE);
            break;
            case 3:
              dxl3.torqueOff(motorID);
              dxl3.writeControlTableItem(PROFILE_VELOCITY, motorID, maxVelocity);
              dxl3.writeControlTableItem(PROFILE_ACCELERATION, motorID, maxAccel);
              dxl3.torqueOn(motorID);
              dxl3.setGoalPosition(motorID, pos, UNIT_DEGREE);
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
        motorID = readByteFromSource(opSource);
        pos = readFloatFromSource(opSource);
        current = readFloatFromSource(opSource);
        if (motorMode[channel][motorID] == 3) {
          switch(channel) {
            case 1:
              dxl1.setGoalCurrent(motorID, current, UNIT_PERCENT);
              dxl1.setGoalPosition(motorID, pos, UNIT_DEGREE);
            break;
            case 2:
              dxl2.setGoalCurrent(motorID, current, UNIT_PERCENT);
              dxl2.setGoalPosition(motorID, pos, UNIT_DEGREE);
            break;
            case 3:
              dxl3.setGoalCurrent(motorID, current, UNIT_PERCENT);
              dxl3.setGoalPosition(motorID, pos, UNIT_DEGREE);
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
        motorID = readByteFromSource(opSource);
        newVelocity = readFloatFromSource(opSource);
        if (motorMode[channel][motorID] == 4) {
          switch(channel) {
            case 1:
              dxl1.setGoalVelocity(motorID, newVelocity, UNIT_RPM);
            break;
            case 2:
              dxl2.setGoalVelocity(motorID, newVelocity, UNIT_RPM);
            break;
            case 3:
              dxl3.setGoalVelocity(motorID, newVelocity, UNIT_RPM);
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
          motorID = USBCOM.readByte();
            switch(channel) {
              case 1:
                pos = dxl1.getPresentPosition(motorID, UNIT_DEGREE);
              break;
              case 2:
                pos = dxl2.getPresentPosition(motorID, UNIT_DEGREE);
              break;
              case 3:
                pos = dxl3.getPresentPosition(motorID, UNIT_DEGREE);
              break;
            }
            USBCOM.writeFloat(pos);
        }
      break;

      case 'M': // Set motor mode
        if (opSource == 0) {
          channel = USBCOM.readByte();
          motorID = USBCOM.readByte();
          newMode = USBCOM.readByte();
          setMotorMode(channel, motorID, newMode);
          USBCOM.writeByte(1);
        }
      break;
    }
  }
}

void startup_motor(uint8_t channel, uint8_t motorIndex) {
  // Connect to motor and initialize in position mode
  switch(channel) {
    case 1:
        // Todo: Add code to find baud rate if motor does not reply
        if (dxl1.ping(motorIndex)) {
          dxl1.setBaudrate(1, 4000000);
        }
    break;
    case 2:
        if (dxl2.ping(motorIndex)) {
          dxl2.setBaudrate(1, 4000000);
        }
    break;
    case 3:
        if (dxl3.ping(motorIndex)) {
          dxl3.setBaudrate(1, 4000000);
        }
    break;
  }
  setMotorMode(channel, motorIndex, 1);
  motorInitialized[channel][motorIndex] = 1;
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
