/* 
NMEA2000_mcp.h

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

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

#include <mcp_can.h>  // https://github.com/Seeed-Studio/CAN_BUS_Shield
#include <NMEA2000.h> 
#include <N2kMsg.h>

class tNMEA2000_mcp : public tNMEA2000
{
private:
  MCP_CAN N2kCAN;

protected:
    bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent=true);
    bool CANOpen();
    bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
    
public:
    tNMEA2000_mcp(unsigned char _N2k_CAN_CS_pin);
};

#endif
