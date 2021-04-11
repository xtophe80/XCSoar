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

#include "util/CharUtil.hxx"
#include "util/StaticFifoBuffer.hxx"
#include "util/CRC.hpp"
//Maybe
#include "util/Compiler.h"

#include <cstdint>

#include <stdio.h>

/**
 * GDL90 device class.
 *
 */
class GDL90Device final : public AbstractDevice {

  static constexpr char FLAG = 0x7E; //!< Message start and end.
  static constexpr char ESC = 0x7D; //!< Control escape.
  static constexpr char HEARTBEAT = 0; //!< Heartbeat message.
  static constexpr size_t HEARTBEAT_LEN = 7;
  static constexpr char UPLINK = 7; //!< Uplink message (FIS-B/Weather).
  static constexpr size_t UPLINK_LEN = 436;
  static constexpr char OWN_GEO_ALT = 11; //!< Position message.
  static constexpr size_t OWN_GEO_ALT_LEN = 5;
  static constexpr char OWNSHIP = 10; //!< Position message.
  static constexpr char TRAFFIC = 20; //!< Traffic message.
  static constexpr size_t TRAFFIC_LEN = 28;
  static constexpr char  BASIC_RPT= 30; //!< Basic UAT report.
  static constexpr size_t BASIC_RPT_LEN = 22;
  static constexpr char LONG_RPT = 31; //!< Long UAT report.
  static constexpr size_t LONG_RPT_LEN = 38;
  static constexpr char FOREFLIGHT = 0x65; //!< Foreflight extension
  static constexpr size_t FOREFLIGHT0_LEN = 39;
  static constexpr size_t FOREFLIGHT1_LEN = 12;



  //! Port the GDL90 device is connected to.
  Port &port;
  //! Expected length of the message just receiving.
  size_t expected_msg_length{};
  //! Buffer which receives the messages send from the radio.
  StaticFifoBuffer<uint8_t, 512u> rx_buf;

public:
  /**
   * Constructor of the radio device class.
   *
   * @param _port Port the GDL90 is connected to.
   */
  GDL90Device(Port &_port);

/**
   * Receives and handles data from the radio.
   *
   * The function parses messages send by the radio.
   * Because all control characters (e.g. STX, ACK, NAK, ...)
   * can be part of the payload of the messages, it is important
   * to separate the messages to distinguish control characters
   * from payload characters.
   */
  virtual bool DataReceived(const void *data, size_t length,
                            struct NMEAInfo &info) override;

};


GDL90Device::GDL90Device(Port &_port)
 : port(_port)
{
}


bool
GDL90Device::DataReceived(const void *_data, size_t length,
                         struct NMEAInfo &info)
{
  size_t message_length = 0;
  size_t consumed = 0;
  uint16_t crc=0;

  assert(_data != nullptr);
  assert(length > 0);

  const uint8_t *data = (const uint8_t *)_data;
  const uint8_t *end = data + length;

  do {
    // Append new data to the buffer, as much as fits in there
    auto range = rx_buf.Write();
    if (rx_buf.IsFull()) {
      // Overflow: reset buffer to recover quickly
      rx_buf.Clear();
      //Potentially mark that we lost sync
      continue;
      }
    size_t nbytes = std::min(range.size, size_t(end - data));
    memcpy(range.data, data, nbytes);
    data += nbytes;
    rx_buf.Append(nbytes);

    for (;;) {
      // Read data from buffer to handle the messages
      range = rx_buf.Read();
      if (range.size < OWNSHIP + 2+2)
	break;
      printf("Range size= %li\n",range.size);
      message_length=0;
      consumed=0;
	if (range.size >= OWN_GEO_ALT_LEN + 2 + 2) {
	  // Search for the start of  message
	  while (*(const uint8_t *) range.data != FLAG && !range.empty()) 
	    {
	    printf("looking\n");
	    range.pop_front();
	    consumed++;
	    }
	  if (range.size < OWN_GEO_ALT_LEN + 2 + 2)
	    break;
	  printf("Found FLAG\n");
	  range.pop_front();
	  if (*(const uint8_t *) range.data == FLAG) {
	    //we caught the end of a message. move 1
	    range.pop_front();
	    consumed++;
	    printf("Followed by another FLAG\n");
	    }
//	  printf("Message ID= %i, 0x%2.2X\n", *(const uint8_t *) range.data, *(const uint8_t *) range.data);
	  switch (*(const uint8_t *) range.data){
	    case HEARTBEAT:
	      {
	      printf("Heatbeat message\n");
	      message_length = HEARTBEAT_LEN;
              //For the time being assume there is no escape characters.
//	  printf("end= 0x%2.2X 0x%2.2X\n", *(const uint8_t *) (range.data + 7), *(const uint8_t *) (range.data + 8));
              if (*(const uint8_t *) (range.data + message_length + 2) != FLAG )
                {printf("Problem\n");
                
                } 
	      crc = UpdateCRC16CCITT((const uint8_t *)range.data, message_length, 0);
              if (*(const uint16_t*) (range.data + message_length) == crc)
                printf("Good CRC\n");
	      printf("CRC %4.4X %4.4X\n", *(const uint16_t*) (range.data + message_length), crc);
	      } 
	      break;
	    case OWNSHIP:
	    case TRAFFIC:
	      {
	      printf("Ownship or traffic message\n");
	      message_length = TRAFFIC_LEN;
              if (*(const uint8_t *) (range.data + message_length + 2) != FLAG )
                printf("Problem\n");
	      crc = UpdateCRC16CCITT((const uint8_t *)range.data, message_length, 0);
              if (*(const uint16_t*) (range.data + message_length) == crc)
                printf("Good CRC\n");
	      }
	      break; 
	    case OWN_GEO_ALT:
	      {
	      printf("Geo altitude message\n");
	      message_length = OWN_GEO_ALT_LEN;
              if (*(const uint8_t *) (range.data + message_length + 2) != FLAG )
                printf("Problem\n");
	      crc = UpdateCRC16CCITT((const uint8_t *)range.data, message_length, 0);
              if (*(const uint16_t*) (range.data + message_length) == crc)
                printf("Good CRC\n");
	      } 
	      break;
	     case FOREFLIGHT:
	       {
	       printf("Foreflight extension\n");
	       if (*(const uint8_t *) (range.data + 1) == 0)
		  message_length = FOREFLIGHT0_LEN;
	       else
		 message_length = FOREFLIGHT1_LEN;
		 // Do nothing
              if (*(const uint8_t *) (range.data + message_length + 2) != FLAG )
                printf("Problem\n");
	      crc = UpdateCRC16CCITT((const uint8_t *)range.data, message_length, 0);
              if (*(const uint16_t*) (range.data + message_length) == crc)
                printf("Good CRC\n");
	       }
	       break;   
	      //We do othing with those messages but it help the parser.
	      case UPLINK:
		message_length = UPLINK_LEN;
		break;
	      case BASIC_RPT:
		message_length = BASIC_RPT_LEN;
		break;
	      case LONG_RPT:
		message_length = LONG_RPT_LEN;
		break;
	      default:
	        break;
	      }
  /*       if (message_length == 0 ){
	     printf( "found nothing. restart\n");
	    range.pop_front();

	     continue;
	   }
  */
	    consumed++ ;
	    if (message_length != 0 && message_length + 3 <= range.size){
	      consumed+= message_length + 3;
	    }
  printf("Range size %li, Message length %li, Consumed %li\n", range.size, message_length, consumed);
	  // Message handled -> remove message
	  rx_buf.Consume(consumed );
	  }   
      } 
  } while (data < end);

  return true;
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


