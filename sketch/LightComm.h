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

#ifndef __LIGHT_COMM_H__
#define __LIGHT_COMM_H__

#include <Arduino.h>
#include <Stream.h>
#include <HardwareSerial.h>

//#define DEBUG_LC

#ifdef DEBUG_LC
#define LC_PRINTLN    _ser->println   
#define LC_PRINT      _ser->print
#else
#define LC_PRINTLN    //_ser->println
#define LC_PRINT      //_ser->print
#endif

#define CHUNK_LEN      10  
//#define DELAY_HIGH     100
//#define DELAY_0        100   //MS
#define DELAY_1        100
//#define DELAY_SLICE    10

enum{
  S_LC_IDLE, S_LC_PREAMBLE0, S_LC_PREAMBLE1, S_LC_DATA0, S_LC_DATA1, S_LC_END
};



class LightComm
{
public:
  LightComm(HardwareSerial *ser); 

  void init(int pin);
  void sample();
  int checkData();
  unsigned char* getData();
  
  

private:
  HardwareSerial *_ser;
  int _pin;
  unsigned char buffer_bin[CHUNK_LEN*9];
  unsigned char buffer_char[CHUNK_LEN+1];
  int _st; 
  long _start;
  int _confirm_cnt ;
  int _last_sample;
  int _bin_len;

};
#endif
