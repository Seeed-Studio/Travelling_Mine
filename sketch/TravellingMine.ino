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



#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <WString.h>

#include <Wire.h>
#include <SeeedOLED.h>
#include "SeeedLogo.h"
#include <TinyGPS.h>
#include <WiFlyHQ.h>
#include "LightComm.h"
#include "SerialCam.h"


#define ST_OFFLINE    1
#define ST_ONLINE     2
#define ST_JOIN       3
#define ST_SCAN       4
#define ST_LIST     5
#define ST_PASSWD     6

int sys_st = ST_JOIN;

int pin_dust = 2;
int pin_hcho = A0;
int pin_mq9 = A3;
int pin_joystick_x = A1;
int pin_joystick_y = A2;
int pin_speaker = 4;
int pin_light = A7;
int pin_wifly_reset = 7;
int pin_wifly_flow_cts = A4;
int pin_wifly_flow_rts = A5;

#define DISPLAY_TOOGLE_INTERVAL    10000   //10s
#define UPLOAD_INTERVAL            60000   //20s

#define UART_GPS    Serial1
#define UART_CAM    Serial2
#define UART_WIFI   Serial3
#define UART_DEBUG  Serial

//gps
TinyGPS gps;
float latitude, longitude;



//wifi
WiFly wifly;
char wifi_ssid[32];
char wifi_pass[32];
char wifi_auth[32];
//join
int wifi_join_cnt = 0;
int wifi_offline_cnt = 0;
//scan wifi
static int scan_phase = 0;
static unsigned long scan_starttime;
struct ssid_s
{
  unsigned char index;
  char name[32];
  //char auth[32];
  struct ssid_s *next;
  struct ssid_s *prev;
};

static struct ssid_s *cur_ssid = NULL;
static struct ssid_s *ssid_head = NULL;

static boolean passwd_inputed = false; 


#define TARGET_IP    "66.117.5.144"
#define TARGET_SITE  "seeedstudio.com"
#define TARGET_PORT  80
#define LOCAL_PORT   9001
#define PIC_UP_URL       "/webservice/wechat/pic_up.php"
#define PM25_PUSH_URL    "/webservice/wechat/pm25.php"
#define WIFLY_FLUSH_TIME  500    // larger than camera read time, avoiding flush too early
#define WIFLY_FLUSH_SIZE  1460   // the max ethernet frame-size

//dust
unsigned long dust_starttime;
unsigned long duration_starttime;
unsigned long duration;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = -1;
int sensorValue=0;

unsigned long display_starttime = 0;
boolean display_dust = true;

unsigned long upload_starttime = 0;

//light
LightComm lm(&Serial);

//camera
SerialCam cam(&UART_CAM);

//joystick
enum{
  K_IDLE, K_PRE_LEFT, K_LEFT, K_PRE_RIGHT, K_RIGHT, K_PRE_UP, K_UP, K_PRE_DOWN, K_DOWN, K_PRE_PRESS, K_PRESS
};
volatile static int sys_key = K_IDLE;
int sys_key_buffed = sys_key;

int key_scan(int last);
void dust_interrupt();

static int timer1_cnt = 0;
static boolean light_recv_stop = true;
ISR(TIMER1_OVF_vect)          //Timer1 Service
{
  if (!light_recv_stop)
  {
    lm.sample();
  }
  
  if (++timer1_cnt == 1)
  {
    timer1_cnt = 0;
    sys_key = key_scan(sys_key); 
    if (sys_key_buffed == K_IDLE && sys_key != K_IDLE && sys_key%2 == 0)
    {
      sys_key_buffed = sys_key;
      //UART_DEBUG.println(sys_key_buffed);
    }
  }
}

#define RESOLUTION 65536    // Timer1 is 16 bit
void init_timer1(long us)
{
  TCCR1A = 0;                 // clear control register A
  TCCR1B = _BV(WGM13);        // set mode as phase and frequency correct pwm, stop the timer

  long cycles;
  long microseconds = us;   //setup microseconds here
  unsigned char clockSelectBits;
  cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if (cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if ((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if ((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if ((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if ((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum

  ICR1 = cycles;
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                          // reset clock select register, and starts the clock

  TIMSK1 = _BV(TOIE1);
  TCNT1 = 0;
  sei();                      //enable global interrupt
}

void init_wifi()
{
  UART_WIFI.begin(9600);
  //delay(1000);
  wifly.reset();
  if(!wifly.begin(&UART_WIFI, &UART_DEBUG))
  {
    UART_DEBUG.println(F("Failed to begin wifly"));
    return;
  }

  wifly.setBaud(115200);
  UART_WIFI.begin(115200);

  delay(1);
  //wifly.setFlushSize(WIFLY_FLUSH_SIZE);     // the max ethernet frame-size
  //wifly.setFlushTimeout(WIFLY_FLUSH_TIME);  //for 1460bytes at 115200 baudrate : 102ms to fill up
  if (pin_wifly_flow_cts > -1)
  {
    UART_DEBUG.println(F("Wifly flow control enabled"));
    wifly.setFlow(true);
  }
  
  wifly.enableDHCP(); 
}

void dust_interrupt()
{
  if (digitalRead(pin_dust) == 0)  //fall
  {
    duration_starttime = millis(); 
    //UART_DEBUG.println(1);
  }else
  {
    duration = millis() - duration_starttime;
    if (duration > 10 && duration < 200)
    {
      //UART_DEBUG.println(duration);
      lowpulseoccupancy+=duration;
    }
    if ((millis()-dust_starttime) > sampletime_ms)
    {
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
      lowpulseoccupancy = 0;
      dust_starttime = millis();
    }
  }
  
}


//******************
void setup() {
  digitalWrite(pin_dust, HIGH);
  pinMode(pin_dust,INPUT);

  pinMode(pin_wifly_reset, OUTPUT);
  digitalWrite(pin_wifly_reset, HIGH);

  pinMode(pin_wifly_flow_cts, INPUT);
  //digitalWrite(pin_wifly_flow_cts, LOW);
  //pinMode(pin_wifly_flow_rts, INPUT/*_PULLUP*/);
  //digitalWrite(pin_wifly_flow_rts, HIGH);

  //tone
  digitalWrite(pin_speaker, LOW);

  
  Wire.begin();
  SeeedOled.init();  //initialze SEEED OLED display
  //SeeedOled.setInverseDisplay();          // Set inverse display
  SeeedOled.clearDisplay();               // clear the screen and set start position to top left corner
  SeeedOled.drawBitmap(SeeedLogo,1024);   // 1024 = 128 Pixels * 64 Pixels / 8
  //delay(2000);

  UART_DEBUG.begin(115200);
  UART_GPS.begin(9600);
  UART_CAM.begin(115200);
  latitude = -1;
  longitude = -1;

  UART_DEBUG.println(F("~~start~~"));
  UART_DEBUG.print(F("Free Ram:"));
  UART_DEBUG.println(wifly.getFreeMemory(),DEC);

  //wifi
  /*strcpy(wifi_ssid, "AndroidAP");
  strcpy(wifi_pass, "11111111");*/
  
  init_wifi();
  
  display_clear();
  SeeedOled.setTextXY(2, 1);
  SeeedOled.putString(F("Joining Wifi")); 
  
  if (analogRead(pin_joystick_x) > 1000)
  {
    sys_st = ST_SCAN;
    scan_phase = 0;
  }
  else if (!wifly.isAssociated()) 
  {
    UART_DEBUG.println(F("Joining the last saved network"));

    //wifly.setSSID(wifi_ssid);
    //wifly.setPassphrase(wifi_pass);
    SeeedOled.setTextXY(4, 1);
    SeeedOled.putString(wifly.getSSID((char *)wifi_ssid, 32)); 
    

    if (wifly.join())
    {
      UART_DEBUG.println(F("Joined wifi network"));
      wifly.save();
      sys_st = ST_ONLINE;
    } else
    {
      UART_DEBUG.println(F("Failed to join wifi network\r\nTry again..."));
      wifi_join_cnt++;
      sys_st = ST_JOIN;
    }
  } else
  {
    UART_DEBUG.println(F("Already joined network"));
    sys_st = ST_ONLINE;
  }

  //sys_st = ST_ONLINE;
  dust_starttime = display_starttime = upload_starttime = millis();
  display_clear();

  init_timer1(10000);   //10ms

  lm.init(pin_light);

  //dust
  attachInterrupt(0, dust_interrupt, CHANGE);

}

//******************
void loop()
{
  switch(sys_st)
  {
    case ST_JOIN:
    {
      process_join();
      sys_key_buffed = K_IDLE; 
      break;
    }
    case ST_SCAN:
    {
      process_scan();
      sys_key_buffed = K_IDLE;
      break;
    }
    case ST_LIST:
    {
      process_list();
      sys_key_buffed = K_IDLE;
      delay(100);
      break;
    }
    case ST_PASSWD:
    {
      process_passwd();
      sys_key_buffed = K_IDLE;
      delay(100);
      break;
    }
    case ST_ONLINE:
    {
      process_online();
      sys_key_buffed = K_IDLE;
      delay(100);
      break;
    }
    default:
    break;
  }
  
  
}

void process_join()
{
  display_clear();
  SeeedOled.setTextXY(3, 1);
  SeeedOled.putString(F("Joining Wifi")); 
  
  wifly.setSSID(wifi_ssid);
  wifly.setPassphrase(wifi_pass); 
  
  if (wifly.join())
  {
    UART_DEBUG.println(F("Joined wifi network"));
    wifly.save();
    sys_st = ST_ONLINE;
    dust_starttime = display_starttime = upload_starttime = millis();
  } else
  {
    if(++wifi_join_cnt >= 3)
    {
      UART_DEBUG.println(F("Failed too many times. Now scan..."));
      sys_st = ST_SCAN;
      scan_phase = false;
      wifi_join_cnt = 0;
    }else{
      UART_DEBUG.println(F("Failed to join wifi network\r\nTry again..."));
      //sys_st = ST_JOIN;
    }
  }
}

int wifi_ap_cnt = 0;
void process_scan()
{
  int ddd = 0;
  if(scan_phase == 0)
  {
    display_clear(); 
    SeeedOled.setTextXY(3,1);
    SeeedOled.putString(F("Scanning Wifi"));

    //free ram
    while (cur_ssid != NULL)
    {
      struct ssid_s *tmp = cur_ssid->next;
      free(cur_ssid);
      cur_ssid = tmp;
    }
    ssid_head = NULL;

    UART_DEBUG.print(F("Free Ram:"));
    UART_DEBUG.println(wifly.getFreeMemory(),DEC);

    do
    {
      wifi_ap_cnt = 0; 
      //wifly.setBaud(9600);
      //UART_WIFI.begin(9600);
      if(!wifly.setopt(PSTR("set sys printlvl 0x4000"), (char *)NULL))
      {
        ddd = 1;
        break;
      }
      if(!wifly.startCommand())
      {
        ddd = 2;
        break;
      }
      wifly.send("scan\r");
      
      scan_phase = 1; 
      scan_starttime = millis();
      return;
    } while (false);
    UART_DEBUG.println(F("Scan failed "));
    UART_DEBUG.println(ddd);
    scan_phase = 0;
  }else if(scan_phase == 1)
  {
    do
    {
      if (!wifly.match("Found ", 10000))
      {
        ddd = 100;
        break;
      }


      char str[128];
      int len;

      if((len = wifly.getsTerm(str, sizeof(str), '\r', 100)) > 0)
      {
        str[len+1] = '\0';
        wifi_ap_cnt = atoi((const char *)str);
        UART_DEBUG.println(wifi_ap_cnt);
      }
      //strip out the title line
      /*if(!wifly.match("Suites", 100))
      {
        ddd = 101;
        break;
      }  */
      unsigned char index = 0;

      while (wifi_ap_cnt-- > 0 && (millis() - scan_starttime) < 10000)
      {
        wifly.match(":", 1000);
        wifly.match(",", 1000);
        if ((len = wifly.getsTerm(str, sizeof(str), '\r', 100)) > 0) 
        {
          struct ssid_s *s = (struct ssid_s *)malloc(sizeof(struct ssid_s));
          strncpy(s->name, str, 31);
          s->index = index++;
          s->prev = cur_ssid;
          s->next = NULL;
          if (cur_ssid != NULL)
          {
            cur_ssid->next = s;
          }else
          {
            ssid_head = s;
          }
          cur_ssid = s;

          UART_DEBUG.println(str); 
          scan_starttime = millis();
        }
        else
        {
          UART_DEBUG.println(".");
        }
      }
      if (wifi_ap_cnt != -1)
      {
        ddd = 102;
        break;
      }
      scan_phase = 2;
      return; 
    } while (false);
    UART_DEBUG.println(F("Parse scan result failed "));
    UART_DEBUG.println(ddd);
    wifly.finishCommand();
    scan_phase = 0;
  }
  else if (scan_phase == 2)
  {
    if (cur_ssid == NULL)
    {
      UART_DEBUG.println(F("Scan result incorrect"));
      sys_st = ST_SCAN;
      scan_phase = 0;
      return;
    }
    wifi_ap_cnt = cur_ssid->index + 1;
    cur_ssid = ssid_head;
    sys_st = ST_LIST;
    display_clear(); 
  }
}

void process_list()
{
  if (cur_ssid == NULL)
  {
    return;
  }

  if (sys_key_buffed == K_DOWN)
  {
    if (cur_ssid->next != NULL)
    {
      cur_ssid = cur_ssid->next;
      display_clear(); 
    }
  }
  if (sys_key_buffed == K_UP)
  {
    if (cur_ssid->prev != NULL)
    {
      cur_ssid = cur_ssid->prev;
      display_clear(); 
    }
  }

  if (sys_key_buffed == K_PRESS)
  {
    sys_st = ST_PASSWD;
    light_recv_stop = false;
    lm.init(pin_light); 
    
    display_clear(); 
    SeeedOled.setTextXY(2,1);
    SeeedOled.putString("Input password");
    passwd_inputed = false;
    return;
  }
  SeeedOled.setTextXY(3,1);
  SeeedOled.putNumber(cur_ssid->index+1);
  SeeedOled.putChar('/');
  SeeedOled.putNumber(wifi_ap_cnt);
  SeeedOled.putChar(' ');
  SeeedOled.putString(cur_ssid->name);
  
}

void process_passwd()
{
  if (sys_key_buffed == K_LEFT || sys_key_buffed == K_RIGHT)
  {
    sys_st = ST_LIST;
    display_clear(); 
    return;
  }
  int len = 0;
  if ((len = lm.checkData()) > 0 && !passwd_inputed) 
  {
    strcpy(wifi_pass, (const char *)lm.getData());

    wifi_pass[len] = '\0';
    SeeedOled.setTextXY(4,1);
    SeeedOled.putString(wifi_pass);
    
    
    UART_DEBUG.println(len);
    UART_DEBUG.println(wifi_pass);
    
  }
  
  if (sys_key_buffed == K_PRESS && cur_ssid != NULL) 
  {
    light_recv_stop = true;
    passwd_inputed = true;
    strcpy(wifi_ssid, (const char *)cur_ssid->name);
    sys_st = ST_JOIN;
  }
}

void process_online()
{
  // ---- co gas
  float vol_CO;
  sensorValue = analogRead(pin_hcho);
  vol_CO=(float)sensorValue/2048*25.0;

  // ---- formal gas
  float vol_Formol;
  sensorValue= analogRead(pin_mq9);
  vol_Formol = (float)sensorValue/1024*1.2;

    // ---- gps
  if (UART_GPS.available())
  {
    while(UART_GPS.available())
    {
      if(gps.encode(UART_GPS.read()))
      {
        gps.f_get_position(&latitude, &longitude);
        UART_DEBUG.print(F("Lat/Long: "));
        UART_DEBUG.print(latitude,5);
        UART_DEBUG.print(", ");
        UART_DEBUG.println(longitude,5);
      }
    }
  }
  //toggle display
  if (millis() - display_starttime > DISPLAY_TOOGLE_INTERVAL || sys_key_buffed == K_LEFT || sys_key_buffed == K_RIGHT)
  {
    display_dust = !display_dust;
    display_clear();
    display_starttime = millis();
  }

  if (display_dust)
  {
    display_dust_gas(concentration/500, vol_Formol, vol_CO);
  }else{
    display_gps(latitude, longitude);
  }

  //camera
  if (sys_key_buffed == K_PRESS)
  {
    display_clear();
    SeeedOled.setTextXY(3, 1);
    SeeedOled.putString(F("Look at camera"));

    play_song("a  a  A");

    shoot_a_photo();

    upload_starttime = millis();
  }

  //upload data
  if (millis() - upload_starttime > UPLOAD_INTERVAL)
  {
    display_clear();
    SeeedOled.setTextXY(3, 1);
    SeeedOled.putString(F("Uploading data"));
    upload_dust_gps(concentration/500, vol_CO, vol_Formol, -1.0f, -1.0f, latitude, longitude);
    upload_starttime = millis();
  }

  //clear the unexpected packets
  //wifly.flushRx(1);

}


int key_scan(int last)
{
  int x = analogRead(pin_joystick_x);
  int y = analogRead(pin_joystick_y);
  if (x > 1000)
  {
    if (last == K_IDLE)
      return K_PRE_PRESS;
    else if (last == K_PRE_PRESS)
      return K_PRESS;
    return last;
  }
  else if (x > 600 && last != K_RIGHT)
  {
    if (last == K_IDLE)
      return K_PRE_RIGHT;
    else if (last == K_PRE_RIGHT)
      return K_RIGHT;
    return last;
  }
  else if (x < 400 && last != K_LEFT)
  {
    if (last == K_IDLE)
      return K_PRE_LEFT;
    else if (last == K_PRE_LEFT)
      return K_LEFT;
    return last;
  }
  else if (y > 600 && last != K_UP)
  {
    if (last == K_IDLE)
      return K_PRE_UP;
    else if (last == K_PRE_UP)
      return K_UP;
    return last;
  }
  else if (y < 400 && last != K_DOWN)
  {
    if (last == K_IDLE)
      return K_PRE_DOWN;
    else if (last == K_PRE_DOWN)
      return K_DOWN;
    return last;
  }
  else
  {
    return K_IDLE;
  }

}


void display_dust_gas(float dust, float formol, float co)
{
  SeeedOled.setTextXY(0,1);
  SeeedOled.putString(F("=====Dust====="));
  SeeedOled.setTextXY(1, 1);
  if (dust < 0) {
    SeeedOled.putString(F("---"));
  }else
    SeeedOled.putFloat(dust, 3);
  SeeedOled.setTextXY(1,7);
  SeeedOled.putString(F("ug/m^3"));


  SeeedOled.setTextXY(3,1);
  SeeedOled.putString(F("==Formol Gas=="));
  SeeedOled.setTextXY(4,1);
  SeeedOled.putFloat(formol);
  SeeedOled.setTextXY(4,7);
  SeeedOled.putString(F("mg/m^3"));


  SeeedOled.setTextXY(6,1);
  SeeedOled.putString(F("====CO Gas===="));
  SeeedOled.setTextXY(7,1);
  SeeedOled.putFloat(co);
  SeeedOled.setTextXY(7,7);
  SeeedOled.putString(F("mg/L"));
}

void display_gps(float latitude, float longitude)
{
  SeeedOled.setTextXY(1, 1);
  SeeedOled.putString(F("=====GPS======"));
  SeeedOled.setTextXY(3, 1);
  SeeedOled.putString(F("LATI:"));
  SeeedOled.setTextXY(3, 7);
  if (latitude < 0)
    SeeedOled.putString(F("no-fix"));
  else
    SeeedOled.putFloat(latitude);
  SeeedOled.setTextXY(5, 1);
  SeeedOled.putString(F("LONG:"));
  SeeedOled.setTextXY(5, 7);
  if (longitude < 0)
    SeeedOled.putString(F("no-fix"));
  else
    SeeedOled.putFloat(longitude);
}

void display_clear()
{
  for (int i = 0; i< 8; i++)
  {
    SeeedOled.setTextXY(i, 0);
    SeeedOled.putString(F("                "));
  }
}

//tone
void freq(int pin, float sec, float freq)
{
  if (freq == 0)
  {
    digitalWrite(pin, LOW);
    delay(1000*sec);
    return;
  }
  float us = 1000000.0f / freq;
  int loops = (int)(sec*1000000.0f/us);
  for (int i=0; i< loops; i++)
  {
    digitalWrite(pin,HIGH);
    delayMicroseconds((int)us/4);
		digitalWrite(pin,LOW);
		delayMicroseconds((int)us/4);
  }
}

void note(char ch)
{
  if (ch==' '     ) freq(pin_speaker, 0.5, 0); // off
  else if (ch=='a') freq(pin_speaker, 0.5, 220.00);
  else if (ch=='b') freq(pin_speaker, 0.5, 246.94);
  else if (ch=='c') freq(pin_speaker, 0.5, 261.63);
  else if (ch=='d') freq(pin_speaker, 0.5, 293.66);
  else if (ch=='e') freq(pin_speaker, 0.5, 329.63);
  else if (ch=='f') freq(pin_speaker, 0.5, 349.23);
  else if (ch=='g') freq(pin_speaker, 0.5, 392.00);
  else if (ch=='A') freq(pin_speaker, 0.5, 440.00);
  else if (ch=='B') freq(pin_speaker, 0.5, 493.88);
  else if (ch=='C') freq(pin_speaker, 0.5, 523.25);
  else if (ch=='D') freq(pin_speaker, 0.5, 587.33);
  else if (ch=='E') freq(pin_speaker, 0.5, 659.26);
  else if (ch=='F') freq(pin_speaker, 0.5, 698.46);
  else if (ch=='G') freq(pin_speaker, 0.5, 783.99);
}

void play_song(char *song)
{
  pinMode(pin_speaker, OUTPUT);
  while (*song != '\0')
  {
    note(*song);
    song++;
  }
  digitalWrite(pin_speaker, LOW); // off
  pinMode(pin_speaker, INPUT);
}

void shoot_a_photo()
{
  int retry = 5;
  int wait_resp = 10000;  //10s
  
  while (retry-- > 0) 
  {
    if (retry == 4)
    {
      cam.init(); 
      cam.capture();
    }

    display_clear();
    SeeedOled.setTextXY(3, 1);
    SeeedOled.putString(F("Uploading photo"));
    SeeedOled.setTextXY(4, 7); 
    SeeedOled.putString("0  %"); 

    if (!http_connect(TARGET_SITE))
    {
      display_clear();
      SeeedOled.setTextXY(3, 1);
      SeeedOled.putString(F("Fail to connect server"));
      delay(2000);
      continue;
    }

    int pktCnt = cam.getPktCnt();
    unsigned long  totalLen = cam.getTotalLen(); 
    UART_DEBUG.println(totalLen);

    http_post_header(PIC_UP_URL, totalLen);
    
    char pkt[PIC_PKT_LEN];
    long len_sum = 0;
    int len = 0;
    for (int i=0; i<pktCnt; i++)
    {
      len = cam.getData(i, pkt);   //wish this will take less time than WIFLY_FLUSH_TIME
      if (len > 0)
      {
        len_sum += len;
        http_post_body(pkt, len);

        SeeedOled.setTextXY(4, 7); 
        SeeedOled.putNumber(100*len_sum/totalLen); 
      }
    }

    while (digitalRead(pin_wifly_flow_cts) == 1);  //wait wifly ready to send

    http_post_end();

    while (digitalRead(pin_wifly_flow_cts) == 1);  //wait wifly ready to send

    UART_DEBUG.println(F("upload done."));
    
    display_clear();
    SeeedOled.setTextXY(3, 1);
    SeeedOled.putString(F("Sync...")); 

    //http_print_response();
    int d = (retry % 2 == 0) ? wait_resp : (wait_resp * 2);
    if (!http_check_response(d))
    {
      UART_DEBUG.println("fail!");
      //play_song("dcba");
    }else
    {
      UART_DEBUG.println("succ!");
      cam.exitCmd();
      play_song("efgA");
      return;
    }

    wifly.flushRx(1);
    display_clear();
    SeeedOled.setTextXY(3, 1);
    SeeedOled.putString(F("Failed,retry...")); 
    delay(2000);
  } ;
  play_song("dcba"); 
  cam.exitCmd(); 
  
}

boolean http_connect(const char *site)
{
  wifi_offline_cnt = 0;
  if (wifly.open(site, 80))
    return true;
  else{
    if (wifly.crashed)
    {
      UART_DEBUG.println(F("reset wifly..."));
      digitalWrite(pin_wifly_reset, LOW);
      delay(1000);
      digitalWrite(pin_wifly_reset, HIGH);
      init_wifi();
    }
    return false;
  }
}

#define boundary    "xxxPOSTBOUNDxxx"
void http_post_header(char *url, unsigned long dataLen)
{
  String start_request = "";
  String end_request = "";
  start_request = start_request 
    + "--"+boundary+ "\r\n" 
    + "Content-Disposition: form-data; name=\"picture\"; filename=\"CAM.JPG\""+"\r\n"
    +"Content-Type: image/jpeg"+"\r\n\r\n";
  end_request = end_request + "\r\n--"+boundary+"--\r\n";
  unsigned long totalLen = dataLen + start_request.length() + end_request.length();

  UART_DEBUG.print(F("New total len:"));
  UART_DEBUG.println(totalLen);

  wifly.print(F("POST "));
  wifly.print(url);
  wifly.println(F(" HTTP/1.1"));
  wifly.print(F("Host: "));
  wifly.println(TARGET_SITE);
  wifly.print(F("Content-Type: multipart/form-data; boundary="));
  wifly.println(boundary);
  wifly.print(F("Content-Length: "));
  wifly.println(totalLen);
  wifly.println(F("Connection: Close"));
  wifly.println();
  wifly.print(start_request);
}

void http_post_body(char *data, int len)
{
  while (len-- > 0)
  {
    while (digitalRead(pin_wifly_flow_cts) == 1);  //wait wifly ready to send
    wifly.write(*data);
    data++;
  }
  //wifly.write((const uint8_t *)data, len); 
}

void http_post_end()
{
  String end_request = "";
  end_request = end_request + "\r\n--"+boundary+"--\r\n";
  wifly.print(end_request);
}

void http_print_response()
{
  UART_DEBUG.println(F("---- http response ----"));
  char c;
  unsigned long t = millis();
  while (wifly.available() == 0)
  {
    if (millis() - t > 10000)
    {
      UART_DEBUG.println(F("timeout!"));
      return;
    }
  }
  while (1)
  {
    if (wifly.gets(&c, 1, 1000) == 0) break;
    UART_DEBUG.write(c);
  }
  UART_DEBUG.println();
}

boolean http_check_response(int delay_ms)
{
  boolean ret = false;
  if (wifly.match("OK", 15000))
  {
    ret = true;
  }
  wifly.flushRx(1);
  return ret;
}

void http_get_header(char *url)
{
  UART_DEBUG.println(url);

  wifly.print(F("GET "));
  wifly.print(url);
  wifly.println(F(" HTTP/1.1"));
  wifly.print(F("host: "));
  wifly.println(TARGET_SITE);
  wifly.println();

}

void upload_dust_gps(float dust, float co, float ch2o, float temp, float humi, float lat, float lon)
{
  if (!http_connect(TARGET_SITE))
  {
    display_clear();
    SeeedOled.setTextXY(3, 1);
    SeeedOled.putString(F("Fail to connect server"));
    delay(2000);
    return;
  }

  UART_DEBUG.println(F("push pm2.5 data..."));
  
  char value[10];

  wifly.print(F("GET "));
  UART_DEBUG.print(F("GET "));
  wifly.print(PM25_PUSH_URL);
  UART_DEBUG.print(PM25_PUSH_URL); 
  
  wifly.print(F("?pm25=")); 
  UART_DEBUG.print(F("?pm25=")); 
  wifly.print(dust,5); 
  UART_DEBUG.print(dust,5); 
  
  wifly.print(F("&co=")); 
  UART_DEBUG.print(F("&co=")); 
  wifly.print(co, 5); 
  UART_DEBUG.print(co, 5); 
  
  wifly.print(F("&ch2o=")); 
  UART_DEBUG.print(F("&ch2o=")); 
  wifly.print(ch2o, 5); 
  UART_DEBUG.print(ch2o, 5); 
  
  wifly.print(F("&temp=")); 
  UART_DEBUG.print(F("&temp=")); 
  wifly.print(temp, 5); 
  UART_DEBUG.print(temp, 5); 
  
  wifly.print(F("&humi=")); 
  UART_DEBUG.print(F("&humi=")); 
  wifly.print(humi, 5); 
  UART_DEBUG.print(humi, 5); 
  
  wifly.print(F("&lati=")); 
  UART_DEBUG.print(F("&lati=")); 
  wifly.print(lat, 5); 
  UART_DEBUG.print(lat, 5); 
  
  wifly.print(F("&longi=")); 
  UART_DEBUG.print(F("&longi=")); 
  wifly.print(lon, 5); 
  UART_DEBUG.println(lon, 5); 

  wifly.println(F(" HTTP/1.1"));
  wifly.print(F("host: "));
  wifly.println(TARGET_SITE);
  wifly.println();

  //while (digitalRead(pin_wifly_flow_cts) == 1);  //wait wifly ready to send

  //http_print_response();
  UART_DEBUG.println(http_check_response(10000)?"ok":"fail");
  wifly.close();

}


