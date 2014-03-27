/*    
 * Copyright (c) 2012 seeed technology inc.  
 * Author      : Jack Shao  
 * Create Time: Mar 2014
 * Change Log : 
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "SerialCam.h"

SerialCam::SerialCam(HardwareSerial *st):_serial(st)
{
  _cameraAddr = (CAM_ADDR << 5);

}


void SerialCam::init()
{
  char cmd[] = {0xaa,0x0d|_cameraAddr,0x00,0x00,0x00,0x00} ;
  unsigned char resp[6];

  _serial->begin(115200);
  _serial->setTimeout(500);
  while (1)
  {
      //clearRxBuf();
      _sendCmd(cmd,6);
      if (_serial->readBytes((char *)resp, 6) != 6)
      {
          continue;
      }
      if (resp[0] == 0xaa && resp[1] == (0x0e | _cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
      {
          if (_serial->readBytes((char *)resp, 6) != 6) continue;
          if (resp[0] == 0xaa && resp[1] == (0x0d | _cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break;
      }
  }
  cmd[1] = 0x0e | _cameraAddr;
  cmd[2] = 0x0d;
  _sendCmd(cmd, 6);
}
void SerialCam::capture()
{
  char cmd[] = { 0xaa, 0x01 | _cameraAddr, 0x00, 0x07, 0x00, PIC_FMT };
  unsigned char resp[6];

  _serial->setTimeout(100);
  while (1)
  {
      clearRxBuf();
      _sendCmd(cmd, 6);
      if (_serial->readBytes((char *)resp, 6) != 6) continue;
      if (resp[0] == 0xaa && resp[1] == (0x0e | _cameraAddr) && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0) break;
  }

  char cmd2[] = { 0xaa, 0x06 | _cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN>>8) & 0xff ,0};
  memcpy(cmd, cmd2, 6);

  _serial->setTimeout(100);
  while (1)
  {
      clearRxBuf();
      _sendCmd(cmd, 6);
      if (_serial->readBytes((char *)resp, 6) != 6) continue;
      if (resp[0] == 0xaa && resp[1] == (0x0e | _cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
  }
  cmd[1] = 0x05 | _cameraAddr;
  cmd[2] = 0;
  cmd[3] = 0;
  cmd[4] = 0;
  cmd[5] = 0;
  while (1)
  {
      clearRxBuf();
      _sendCmd(cmd, 6);
      if (_serial->readBytes((char *)resp, 6) != 6) continue;
      if (resp[0] == 0xaa && resp[1] == (0x0e | _cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
  }
  cmd[1] = 0x04 | _cameraAddr;
  cmd[2] = 0x1;
  while (1)
  {
      clearRxBuf();
      _sendCmd(cmd, 6);
      if (_serial->readBytes((char *)resp, 6) != 6) continue;
      if (resp[0] == 0xaa && resp[1] == (0x0e | _cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
      {
          _serial->setTimeout(1000);
          if (_serial->readBytes((char *)resp, 6) != 6)
          {
              continue;
          }
          if (resp[0] == 0xaa && resp[1] == (0x0a | _cameraAddr) && resp[2] == 0x01)
          {
              _picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
              //_serial->print("_picTotalLen:");
              //_serial->println(_picTotalLen);
              break;
          }
      }
  }


}

int SerialCam::getPktCnt()
{
  unsigned int pktCnt = (_picTotalLen) / (PIC_PKT_LEN - 6);
  if ((_picTotalLen % (PIC_PKT_LEN-6)) != 0) pktCnt += 1;
  _pktCnt = pktCnt;
  return pktCnt;
}

unsigned long  SerialCam::getTotalLen()
{
  return _picTotalLen;
}

int SerialCam::getData(int i, char *dest)
{
  
  char cmd[] = { 0xaa, 0x0e | _cameraAddr, 0x00, 0x00, 0x00, 0x00 };
  unsigned char pkt[PIC_PKT_LEN];

  _serial->setTimeout(1000);
  cmd[4] = i & 0xff;
  cmd[5] = (i >> 8) & 0xff;

  int retry_cnt = 0;
  retry:
  delay(10);
  clearRxBuf();
  _sendCmd(cmd, 6);
  int cnt = _serial->readBytes((char *)pkt, PIC_PKT_LEN);

  unsigned char sum = 0;
  for (int y = 0; y < cnt - 2; y++)
  {
      sum += pkt[y];
  }
  if (sum != pkt[cnt-2])
  {
      if (++retry_cnt < 100) goto retry;
      else return 0;
  }

  //myFile.write((const uint8_t *)&pkt[4], cnt-6);
  memcpy(dest, (const uint8_t *)&pkt[4], cnt-6);
  //if (cnt != PIC_PKT_LEN) break;

  /*if (i == (_pktCnt-1))
  {
    cmd[4] = 0xf0; 
    cmd[5] = 0xf0;
    _sendCmd(cmd, 6);
  }*/
  return cnt-6;
}

void SerialCam::exitCmd()
{
  char cmd[] = { 0xaa, 0x0e | _cameraAddr, 0x00, 0x00, 0xf0, 0xf0 };
  unsigned char resp[6];

  _sendCmd(cmd, 6); 
}
void SerialCam::clearRxBuf()
{
  while (_serial->available())
  {
      _serial->read();
  }
}


void SerialCam::_sendCmd(char cmd[], int cmd_len)
{
    for (char i = 0; i < cmd_len; i++) _serial->write(cmd[i]);
}
