/*
NMEA2000_mcp.cpp

Copyright (c) 2015-2017 Timo Lappalainen, Kave Oy, www.kave.fi

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

struct tCANFrame {
  uint32_t id; // can identifier
  uint8_t len; // length of data
  uint8_t buf[8];
};

bool CanInUse=false;
tNMEA2000_mcp *pNMEA2000_mcp1=0;

void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
void Can1Interrupt();

//*****************************************************************************
void PrintDecodedCanIdAndLen(unsigned long id, unsigned char len) {
  unsigned char prio;
  unsigned long pgn;
  unsigned char src;
  unsigned char dst;

  if (id!=0) {
    CanIdToN2k(id,prio,pgn,src,dst);
    Serial.print(millis());
    Serial.print(": pgn: "); Serial.print(pgn); Serial.print(", prio: "); Serial.print(prio);
    Serial.print(", src: "); Serial.print(src); Serial.print(", dst: "); Serial.print(dst);
  } else {
    Serial.print("id: "); Serial.print(id);
  }
  Serial.print(", len: "); Serial.println(len);
}

//*****************************************************************************
tNMEA2000_mcp::tNMEA2000_mcp(unsigned char _N2k_CAN_CS_pin, unsigned char _N2k_CAN_clockset,
                             unsigned char _N2k_CAN_int_pin, uint16_t _rx_frame_buf_size) : tNMEA2000(), N2kCAN() {
  // CanIntChk=0;

  IsOpen=false;
  N2k_CAN_CS_pin=_N2k_CAN_CS_pin;
  N2k_CAN_clockset=_N2k_CAN_clockset;
  if (pNMEA2000_mcp1==0) { // Currently only first instance can use interrupts.
    N2k_CAN_int_pin=_N2k_CAN_int_pin;
    if ( UseInterrupt() ) {
      MaxCANReceiveFrames=_rx_frame_buf_size;
      pNMEA2000_mcp1=this;
    }
  } else {
    N2k_CAN_int_pin=0xff;
  }
}

//*****************************************************************************
bool tNMEA2000_mcp::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
  bool result;

    // Also sending should be changed to be done by interrupt. This requires modifications for mcp_can.
    uint8_t SaveSREG = SREG;   // save interrupt flag
    volatile tFrameBuffer *pTxBuf=0;
    if ( UseInterrupt() ) {
      cli();   // disable interrupts
      pTxBuf=(wait_sent?pTxBufferFastPacket:pTxBuffer);
      // If buffer is not empty, it has pending messages, so add new message to it
      if ( !pTxBuf->IsEmpty() ) {
        result=pTxBuf->AddFrame(id,len,buf);
      } else { // If we did not use buffer, send it directly
        result=(N2kCAN.trySendMsgBuf(id, 1, len, buf, wait_sent?N2kCAN.getLastTxBuffer():0xff)==CAN_OK);
        if ( !result ) result=pTxBuf->AddFrame(id,len,buf);
      }
      SREG = SaveSREG;   // restore the interrupt flag
    } else {
      result=(N2kCAN.trySendMsgBuf(id, 1, len, buf, wait_sent?N2kCAN.getLastTxBuffer():0xff)==CAN_OK);
    }

    // Serial.println(result);
    // if ( CanIntChk ) { Serial.print("CAN int chk: "); Serial.println(CanIntChk); CanIntChk=0; }

    return result;
}

//*****************************************************************************
void tNMEA2000_mcp::InitCANFrameBuffers() {
    if ( UseInterrupt() ) {
      if (MaxCANReceiveFrames<2 ) MaxCANReceiveFrames=2;
      if (MaxCANSendFrames<10 ) MaxCANSendFrames=10;
      uint16_t CANGlobalBufSize=MaxCANSendFrames-4;
      MaxCANSendFrames=4;  // we do not need much libary internal buffer since driver has them.
      uint16_t FastPacketBufferSize= (CANGlobalBufSize * 9 / 10);
      CANGlobalBufSize-=FastPacketBufferSize;
      pRxBuffer=new tFrameBuffer(MaxCANReceiveFrames);
      pTxBuffer=new tFrameBuffer(CANGlobalBufSize);
      pTxBufferFastPacket=new tFrameBuffer(FastPacketBufferSize);
    }

    tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

//*****************************************************************************
bool tNMEA2000_mcp::CANOpen() {
    if (IsOpen) return true;

    if (CanInUse) return false; // currently prevent accidental second instance. Maybe possible in future.

    N2kCAN.init_CS(N2k_CAN_CS_pin);
    N2kCAN.reserveTxBuffers(1); // Reserve one buffer for fast packet.
    IsOpen=(N2kCAN.begin(CAN_250KBPS,N2k_CAN_clockset)==CAN_OK);

    if (IsOpen && UseInterrupt() ) {
      N2kCAN.enableTxInterrupt();
      attachInterrupt(digitalPinToInterrupt(N2k_CAN_int_pin), Can1Interrupt, FALLING);
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
      HasFrame=pRxBuffer->GetFrame(id,len,buf);
      SREG = SaveSREG;   // restore the interrupt flag
    } else {
      if ( CAN_MSGAVAIL == N2kCAN.checkReceive() ) {           // check if data coming
          N2kCAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
          id = N2kCAN.getCanId();

          HasFrame=true;
      }
    }

    // if (HasFrame) PrintDecodedCanIdAndLen(id,len);

    return HasFrame;
}

//*****************************************************************************
// I am still note sure am I handling volatile right here since mcp_can has not
// been defined volatile. see. http://blog.regehr.org/archives/28
// In my tests I have used only to receive data or transmit data but not both.
void tNMEA2000_mcp::InterruptHandler() {
  INT8U RxTxStatus;
  // Iterate over all pending messages.
  // If either the bus is saturated or the MCU is busy, both RX buffers may be in use and
  // reading a single message does not clear the IRQ conditon.
  // Also we need to check and clear all transmit flags to clear IRQ condition.
  // Note that this handler expects that Wakeup and Error interrupts has not been enabled.
  do {
    uint32_t id;
    unsigned char len;
    unsigned char buf[8];

    RxTxStatus=N2kCAN.readRxTxStatus();  // One single read on every loop
    INT8U tempRxTxStatus=RxTxStatus;      // Use local status inside loop

    while ( N2kCAN.checkClearRxStatus(&tempRxTxStatus)!=0 ) {           // check if data is coming
      N2kCAN.readMsgBuf(&len,buf);
      id=N2kCAN.getCanId();
//      asm volatile ("" : : : "memory");
      pRxBuffer->AddFrame(id,len,buf);
    }

    if ( !pTxBuffer->IsEmpty() ) { // Do we have something to send on single frame buffer
      while ( N2kCAN.checkClearTxStatus(&tempRxTxStatus)!=0 && pTxBuffer->GetFrame(id,len,buf) ) {
        N2kCAN.trySendMsgBuf(id, 1, len, buf);
      }
    } else { // Nothing to send, so clear flags
      N2kCAN.clearBufferTransmitIfFlags();
    }

    if ( !pTxBufferFastPacket->IsEmpty() ) { // Do we have something to send on fast packet frame buffer
      // CanIntChk=tempRxTxStatus;
      if ( N2kCAN.checkClearTxStatus(&tempRxTxStatus,N2kCAN.getLastTxBuffer())!=0 ) {
        pTxBufferFastPacket->GetFrame(id,len,buf);
        N2kCAN.trySendMsgBuf(id, 1, len, buf,N2kCAN.getLastTxBuffer());
      }
    } else { // Nothing to send, so clear flag for this buffer
      N2kCAN.clearBufferTransmitIfFlags(N2kCAN.getLastTxBuffer());
    }

  } while ( RxTxStatus!=0 );
}

//*****************************************************************************
void Can1Interrupt() {
  pNMEA2000_mcp1->InterruptHandler();
}
