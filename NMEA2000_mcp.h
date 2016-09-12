/* 
NMEA2000_mcp.h

2015-2016 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
  
Inherited NMEA2000 object for Arduino CAN_BUS shield or any MCP2515 CAN controller
based setup. See also NMEA2000 library.
*/

#ifndef _NMEA2000_MCP_H_
#define _NMEA2000_MCP_H_

// CAN_BUS_shield libraries will be originally found on https://github.com/Seeed-Studio/CAN_BUS_Shield
// That does not work completely with N2k or with Maple mini. So there is developed
// branch found on https://github.com/peppeve/CAN_BUS_Shield 
#include <mcp_can.h>
#include <NMEA2000.h> 
#include <N2kMsg.h>

// Define size of 
#ifndef MCP_CAN_RX_BUFFER_SIZE
#define MCP_CAN_RX_BUFFER_SIZE 50 
#endif

class tNMEA2000_mcp : public tNMEA2000
{
private:
  MCP_CAN N2kCAN;
  unsigned char N2k_CAN_CS_pin;
  unsigned char N2k_CAN_clockset;
  unsigned char N2k_CAN_int_pin;
  bool IsOpen;
  
protected:
    bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent=true);
    bool CANOpen();
    bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
    bool UseInterrupt() { return N2k_CAN_int_pin!=0xff; }
    
public:
    tNMEA2000_mcp(unsigned char _N2k_CAN_CS_pin, unsigned char _N2k_CAN_clockset = MCP_16MHz, 
                  unsigned char _N2k_CAN_int_pin = 0xff, uint16_t rx_frame_buf_size=MCP_CAN_RX_BUFFER_SIZE);
    void SetSPI(SPIClass *_pSPI) { N2kCAN.setSPI(_pSPI); }
};

#endif
