/* 
NMEA2000_mcp.cpp

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
*/

#include <NMEA2000_mcp.h> 

//*****************************************************************************
tNMEA2000_mcp::tNMEA2000_mcp(unsigned char _N2k_CAN_CS_pin, unsigned char _N2k_CAN_clockset) : tNMEA2000(), N2kCAN() {
  N2k_CAN_CS_pin=_N2k_CAN_CS_pin;
  N2k_CAN_clockset=_N2k_CAN_clockset;
}

//*****************************************************************************
bool tNMEA2000_mcp::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
  INT8U result;

    result=N2kCAN.sendMsgBuf(id, 1, len, buf,wait_sent);
//    Serial.println(result);
    return (result==CAN_OK); 
}

//*****************************************************************************
bool tNMEA2000_mcp::CANOpen() {
    N2kCAN.init_CS(N2k_CAN_CS_pin);
    return (N2kCAN.begin(CAN_250KBPS,N2k_CAN_clockset)==CAN_OK);
}

//*****************************************************************************
bool tNMEA2000_mcp::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;

    if ( CAN_MSGAVAIL == N2kCAN.checkReceive() ) {           // check if data coming
        N2kCAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        id = N2kCAN.getCanId();
        
        HasFrame=true;
    }
    
    return HasFrame;
}
