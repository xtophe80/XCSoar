/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2021 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Device/Driver/GDL90.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "NMEA/Info.hpp"

//Maybe
#include "util/CharUtil.hxx"
#include "util/StaticFifoBuffer.hxx"
#include "util/Compiler.h"

#include <cstdint>

#include <stdio.h>

/**
 * GDL90 device class.
 *
 * This class provides the interface to communicate with the KRT2 radio.
 * The driver retransmits messages in case of a failure.
 * See
 * http://bugs.xcsoar.org/raw-attachment/ticket/2727/Remote_control_Interface_V12.pdf
 * for the protocol specification.
 */
class GDL90Device final : public AbstractDevice {
  static constexpr auto CMD_TIMEOUT = std::chrono::milliseconds(250); //!< Command timeout
  static constexpr unsigned NR_RETRIES = 3; //!< Number of tries to send a command.

  static constexpr char STX = 0x02; //!< Command start character.
  static constexpr char ACK = 0x06; //!< Command acknowledged character.
  static constexpr char NAK = 0x15; //!< Command not acknowledged character.
  static constexpr char NO_RSP = 0; //!< No response received yet.

  static constexpr size_t MAX_NAME_LENGTH = 8; //!< Max. radio station name length.


  //! Port the GDL90 device is connected to.
  Port &port;

public:
  /**
   * Constructor of the radio device class.
   *
   * @param _port Port the radio is connected to.
   */
  GDL90Device(Port &_port);

};


GDL90Device::GDL90Device(Port &_port)
 : port(_port)
{
}

static Device *
GDL90CreateOnPort(const DeviceConfig &config, Port &comPort)
{
  Device *dev = new GDL90Device(comPort);

  return dev;
}


const struct DeviceRegister gdl90_driver = {
  _T("GDL90"),
  _T("GDL90"),
  DeviceRegister::RAW_GPS_DATA,
  GDL90CreateOnPort,
};


