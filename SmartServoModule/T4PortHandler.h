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
// T4PortHandler is a custom serial port handler configured to use Teensy 4's optimized RS-485 dir pin control.
// This approach reduces blocking execution time for a typical register write instruction at 4Mb/s from ~50us to ~5us.
// Note: A line must be included in the setup() function of the main sketch: SerialX.transmitterEnable(Y); 
//       where X is the hardware serial channel and Y is the channel's DIR pin.
// This code is modified from an example in the Robotis Dynamixel2Arduino library:
// github.com/ROBOTIS-GIT/Dynamixel2Arduino/blob/master/examples/advanced/add_custom_SerialPortHandler/add_custom_SerialPortHandler.ino

class T4PortHandler : public DYNAMIXEL::SerialPortHandler
{
  public:
    T4PortHandler(HardwareSerial& port, const int dir_pin = -1)
    : SerialPortHandler(port, dir_pin), port_(port), dir_pin_(dir_pin)
    {}

    virtual size_t write(uint8_t c) override
    {
      size_t ret = 0;
      ret = port_.write(c);
      return ret;
    }

    virtual size_t write(uint8_t *buf, size_t len) override
    {
      size_t ret;
      ret = port_.write(buf, len);
      return ret;
    }

  private:
    HardwareSerial& port_;
    const int dir_pin_;
};
