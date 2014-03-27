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

#ifndef __SERIALCAM_H__
#define __SERIALCAM_H__

#include <Arduino.h>
#include <Stream.h>
#include <HardwareSerial.h>

#define PIC_PKT_LEN    128                  //data length of each read, dont set this too big because ram is limited
#define CAM_ADDR       0

#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define PIC_FMT        PIC_FMT_VGA



class SerialCam
{
public:
  SerialCam(HardwareSerial *);

  void init();
  void capture();
  int getPktCnt();
  unsigned long  getTotalLen(); 
  int getData(int idx, char *dest);
  void exitCmd();
  void clearRxBuf();

private:
  HardwareSerial *_serial;
  byte _cameraAddr;
  unsigned long _picTotalLen;
  int _pktCnt;

  void _sendCmd(char [], int cmd_len);
};
#endif
