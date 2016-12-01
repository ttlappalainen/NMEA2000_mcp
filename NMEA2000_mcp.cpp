/* 
NMEA2000_mcp.cpp

Copyright (c) 2015-2016 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "NMEA2000_mcp.h" 


typedef struct tCANFrame {
  uint32_t id; // can identifier
  uint8_t len; // length of data
  uint8_t buf[8];
} CAN_message_t;

bool CanInUse=false; 
MCP_CAN *pN2kCAN=0;
volatile tCANFrame *rx_frame_buff=0;
volatile uint16_t rx_frame_buf_size=0;
volatile uint8_t rx_buffer_read=0; 
volatile uint8_t rx_buffer_write=0;

void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
void CanInterrupt();

//*****************************************************************************
void PrintDecodedCanIdAndLen(unsigned long id, unsigned char len) {
  unsigned char prio;
  unsigned long pgn;
  unsigned char src;
  unsigned char dst;
  
  if (id!=0) {
    CanIdToN2k(id,prio,pgn,src,dst);
    Serial.print("pgn: "); Serial.print(pgn); Serial.print(", prio: "); Serial.print(prio);
    Serial.print(", src: "); Serial.print(src); Serial.print(", dst: "); Serial.print(dst); 
  } else {
    Serial.print("id: "); Serial.print(id);
  }
  Serial.print(", len: "); Serial.println(len);
}

//*****************************************************************************
tNMEA2000_mcp::tNMEA2000_mcp(unsigned char _N2k_CAN_CS_pin, unsigned char _N2k_CAN_clockset, 
                             unsigned char _N2k_CAN_int_pin, uint16_t _rx_frame_buf_size) : tNMEA2000(), N2kCAN() {
  IsOpen=false;
  N2k_CAN_CS_pin=_N2k_CAN_CS_pin;
  N2k_CAN_clockset=_N2k_CAN_clockset;
  if (pN2kCAN==0) { // Currently only first instace can use interrupts.
    N2k_CAN_int_pin=_N2k_CAN_int_pin;
    if ( UseInterrupt() ) {
      rx_frame_buf_size=_rx_frame_buf_size;
      if (rx_frame_buf_size<2) rx_frame_buf_size=2;
      rx_frame_buff=new tCANFrame[rx_frame_buf_size];
      pN2kCAN=&N2kCAN;
    }
  } else {
    N2k_CAN_int_pin=0xff;
  }
}

//*****************************************************************************
bool tNMEA2000_mcp::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
  INT8U result;

    // Also sending should be changed to be done by interrupt. This requires modifications for mcp_can.
    uint8_t SaveSREG = SREG;   // save interrupt flag
    if ( UseInterrupt() ) cli();   // disable interrupts
    result=N2kCAN.sendMsgBuf(id, 1, len, buf,wait_sent);
    if ( UseInterrupt() ) SREG = SaveSREG;   // restore the interrupt flag
//    Serial.println(result);
    return (result==CAN_OK); 
}

//*****************************************************************************
bool tNMEA2000_mcp::CANOpen() {
    if (IsOpen) return true;
    
    if (CanInUse) return false; // currently prevent accidental second instance. Maybe possible in future.
    
    N2kCAN.init_CS(N2k_CAN_CS_pin);
    IsOpen=(N2kCAN.begin(CAN_250KBPS,N2k_CAN_clockset)==CAN_OK);
    
    if (IsOpen && UseInterrupt() ) {
      rx_buffer_read=0;
      rx_buffer_write=0;
      attachInterrupt(digitalPinToInterrupt(N2k_CAN_int_pin), CanInterrupt, FALLING);
    }    
    
    CanInUse=IsOpen;
    
    return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_mcp::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;

    if ( UseInterrupt() ) {
      uint8_t SaveSREG = SREG;   // save interrupt flag
      cli();   // disable interrupts
      if (rx_buffer_read!=rx_buffer_write) {
//        Serial.print("read: "); Serial.print(rx_buffer_read); Serial.print(", write: "); Serial.println(rx_buffer_write);

        id = rx_frame_buff[rx_buffer_read].id;
        len = rx_frame_buff[rx_buffer_read].len;
        for (int i=0; i<len; buf[i]=rx_frame_buff[rx_buffer_read].buf[i], i++);
        rx_buffer_read = (rx_buffer_read + 1) % rx_frame_buf_size;
        HasFrame=( (id!=0) && (len!=0) );      
      }
      SREG = SaveSREG;   // restore the interrupt flag
    } else {
      if ( CAN_MSGAVAIL == N2kCAN.checkReceive() ) {           // check if data coming
          N2kCAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
          id = N2kCAN.getCanId();
          
          HasFrame=true;
      }
    }
    
//    if (HasFrame) PrintDecodedCanIdAndLen(id,len);
    
    return HasFrame;
}

//*****************************************************************************
// I am still note sure am I handling volatile right here since mcp_can has not
// been defined volatile. see. http://blog.regehr.org/archives/28
// In my tests I have used only to receive data or transmit data but not both.
void CanInterrupt() {
  // Iterate over all pending messages.
  // If either the bus is saturated or the MCU is busy, both RX buffers may be in use and
  // reading a single message does not clear the IRQ conditon.
  while ( CAN_MSGAVAIL == pN2kCAN->checkReceive() ) {           // check if data coming
    uint8_t temp = (rx_buffer_write + 1) % rx_frame_buf_size;
    uint32_t id;
    unsigned char len;
    unsigned char buf[8];

    pN2kCAN->readMsgBuf(&len,buf);
    id=pN2kCAN->getCanId();
    asm volatile ("" : : : "memory");
    if ( (temp != rx_buffer_read) && (len<=8) ) { // check is there room for new message. If not, we loose it.
      rx_frame_buff[rx_buffer_write].id=id;
      rx_frame_buff[rx_buffer_write].len=len;
      for (int i=0; i<len; rx_frame_buff[rx_buffer_write].buf[i]=buf[i], i++);
      rx_buffer_write = temp;
    }
  }
}
