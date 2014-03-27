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


#include "LightComm.h"

LightComm::LightComm(HardwareSerial *ser)
{
  _ser = ser;
}

void LightComm::init(int pin)
{
  _pin = pin;
  pinMode(pin, INPUT);

  _st = S_LC_IDLE;
  _start = millis();
  _confirm_cnt = 0;
  _last_sample = 1;

  memset(buffer_bin, 0, sizeof(buffer_bin));
  memset(buffer_char, 0, sizeof(buffer_char));

}

void LightComm::sample()
{
  int sample = analogRead(_pin) > 500 ? 1:0;  //1: white 0:black
  
  switch (_st)
  {
  case S_LC_IDLE:
    {
      _bin_len = 0;
      if (_last_sample == 0 && sample == 1)
      {
        _start = millis(); 
        _st = S_LC_PREAMBLE0;
        LC_PRINTLN("**");
      }
      break; 
    }
  case S_LC_PREAMBLE0:
    {
      //wait fall
      if (sample == 1)
      {
        if (millis() - _start > 1500) //timeout
        {
          _st = S_LC_IDLE;
        }
      } else
      {
        if (millis() - _start > 500)
        {
          _start = millis();
          _st = S_LC_PREAMBLE1;
          LC_PRINTLN("P1"); 
        }
        else _st = S_LC_IDLE;
      }
      
      break; 
    }
  case S_LC_PREAMBLE1:
    {
      //wait rise
      if (sample == 0)
      {
        if (millis() - _start > 1500) //timeout
        {
          _st = S_LC_IDLE;
        }
      } else
      {
        if (millis() - _start > 500)
        {
          _start = millis();
          _st = S_LC_DATA0;
          _bin_len = 0;
          LC_PRINTLN("D0"); 
        }
        else _st = S_LC_IDLE;
      }

      break;
    }
  case S_LC_DATA0:
    {
      if (sample == 1)  //end
      {
        if (millis() - _start > 1500)
        {
          _st = S_LC_END;
          LC_PRINTLN("END"); 
        }
      } else
      {
        _start = millis();
        _st = S_LC_DATA1;
        LC_PRINTLN("D1"); 
      }
      break; 
    }
  case S_LC_DATA1:
    {
      if (sample == 0)
      {
        if (millis() - _start > 1500) //timeout
        {
          _st = S_LC_IDLE;
          LC_PRINTLN("TO D1"); 
        }
      }
      else
      {
        long d = millis() - _start;
        if (d > DELAY_1)  //1
        {
          buffer_bin[_bin_len++] = 1;
          _st = S_LC_DATA0;
          LC_PRINTLN("1"); 
        }
        else//0
        {
          buffer_bin[_bin_len++] = 0; 
          _st = S_LC_DATA0; 
          LC_PRINTLN("0"); 
        }
        
      }
      break; 
    }
  case S_LC_END:
    {
      break;
    }
  }
  
  _last_sample = sample;
}

int LightComm::checkData()
{
  if (_bin_len > 0 && _bin_len%9 == 0)
  {
    int cnt = _bin_len / 9;
    cnt = min(cnt, CHUNK_LEN); 
    unsigned char sum = 0;
    unsigned char c = 0;
    for (int i = 0; i < cnt;i++) 
    {
      sum = 0;
      for (int j = 0; j < 8;j++) 
      {
        c = (c << 1) | (buffer_bin[9 * i + j] & 0x1);
        sum += (buffer_bin[9 * i + j] & 0x1);
        sum &= 0x1;
      }
      if (sum != buffer_bin[9*i+8])
      {
        LC_PRINTLN("crc check fail");
        return 0;
      }
      buffer_char[i] = c;
    }
    buffer_char[cnt] = '\0';
    LC_PRINTLN((char *)buffer_char);
    return cnt;
  }
  else
  {
     LC_PRINT("Len error: ");
     LC_PRINTLN(_bin_len);
     return 0;
  }
}

unsigned char* LightComm::getData()
{
  return buffer_char;
}
