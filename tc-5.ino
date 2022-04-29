//MAX7219 LED display
#include "LedControl.h"

#define LED_MODE 2 //1 for HH.MM.SS.FF, 2 for HH-MM-SS

#define LED_BUILTIN 13  //for MiniEVB

//nano: 16 Mhz
//timer divider by 8 = 2 MHz
//timer's pulse: 0,5 us
#define icpPin 8            // ICP input pin on arduino

#define LED_DIN 10
#define LED_CS 11
#define LED_CLK 12

LedControl lc = LedControl(LED_DIN, LED_CLK, LED_CS, 1);  // 1 as we are only using 1 MAX7219

/*margins
  PAL = 25FPS = 40ms per frame
  40ms / 80bits = 500us per bit
  250us per half bit
  125us per quarter bit
  0 bit max = 625us
  1 bit max = 375us (also 0 bit min)
  1 bit min = 125us

  125 -- 250 -- 375 -- 500 -- 625
  min1    1     max1    0    max0

  NTSC 33.3667 ms/frame
  417.1 us per bit
  208.5 us per half bit
  104.3 us per quarter bit

  NTSC margins
  125 -- 209 -- 375 -- 417 -- 625
*/

//timecode values
#define zero_time_max 1250  //625 uS
#define zero_time_min 750   //375 uS
#define one_time_max  750   //the same, actually this is not used in this sketch
#define one_time_min  250   //125 uS

#define end_data_position  63
#define end_sync_position  77
#define end_smpte_position 80

volatile unsigned int bit_time;
volatile boolean valid_tc_word;
volatile boolean ones_bit_count;
volatile boolean tc_sync;
volatile boolean write_tc_out;
volatile boolean drop_frame_flag;

volatile byte total_bits;
volatile byte current_bit;
volatile byte sync_count;

volatile byte tc[8];
volatile byte newtc[8];
volatile char timeCode[11];

bool led_state = false;

/* ICR interrupt vector */
ISR(TIMER1_CAPT_vect)
{
  //toggle capture edge
  TCCR1B ^= _BV(ICES1);

  bit_time = ICR1;

  //reset timer1
  TCNT1 = 0;

  if ((bit_time < one_time_min) || (bit_time > zero_time_max)) // get rid of anything way outside the norm
  {
    //Serial.println(bit_time, DEC); //debug
    total_bits = 0;
    //toggle built-in led in case of error
    digitalWrite(LED_BUILTIN, (led_state) ? HIGH : LOW);
    led_state = !led_state;
  }
  else
  {
    if (ones_bit_count == true) // only count the second ones pulse
      ones_bit_count = false;
    else
    {
      if (bit_time > zero_time_min)
      {
        current_bit = 0;
        sync_count = 0;
      }
      else //if (bit_time < one_time_max)
      {
        ones_bit_count = true;
        current_bit = 1;
        sync_count++;
        if (sync_count == 12) // part of the last two bytes of a timecode word
        {
          sync_count = 0;
          tc_sync = true;
          total_bits = end_sync_position;
        }
      }

      if (total_bits <= end_data_position) // timecode runs least to most so we need
      { // to shift things around
        tc[0] = tc[0] >> 1;

        for (int n = 1; n < 8; n++)
        {
          if (tc[n] & 1)
            tc[n - 1] |= 0x80;
          tc[n] = tc[n] >> 1;
        }

        if (current_bit == 1)
          tc[7] |= 0x80;
      }
      total_bits++;
    }

    if (total_bits == end_smpte_position) // we have the 80th bit
    {
      total_bits = 0;
      if (tc_sync)
      {
        tc_sync = false;
        valid_tc_word = true;
      }
    }

    if (valid_tc_word)
    {
      valid_tc_word = false;

      newtc[0] = tc[0] & 0x0F;
      newtc[1] = tc[1] & 0x03;
      newtc[2] = tc[2] & 0x0F;
      newtc[3] = tc[3] & 0x07;
      newtc[4] = tc[4] & 0x0F;
      newtc[5] = tc[5] & 0x07;
      newtc[6] = tc[6] & 0x0F;
      newtc[7] = tc[7] & 0x03;

      /*      timeCode[10] = (tc[0] & 0x0F) + 0x30;  // frames
            timeCode[9] = (tc[1] & 0x03) + 0x30;  // 10's of frames
            timeCode[8] =  '.';
            timeCode[7] = (tc[2] & 0x0F) + 0x30;  // seconds
            timeCode[6] = (tc[3] & 0x07) + 0x30;  // 10's of seconds
            timeCode[5] =  ':';
            timeCode[4] = (tc[4] & 0x0F) + 0x30;  // minutes
            timeCode[3] = (tc[5] & 0x07) + 0x30;  // 10's of minutes
            timeCode[2] = ':';
            timeCode[1] = (tc[6] & 0x0F) + 0x30;  // hours
            timeCode[0] = (tc[7] & 0x03) + 0x30;  // 10's of hours
      */
      drop_frame_flag = bit_is_set(tc[1], 2);

      write_tc_out = true; // allow to output the timecode
    }
  }
} // of ISR


void setup()
{
  Serial.begin(115200);

  // display set up
  // the zero refers to the MAX7219 number, it is zero for 1 chip
  lc.shutdown(0, false); // turn off power saving, enables display
  lc.setIntensity(0, 6); // sets brightness (0~15 possible values)
  lc.clearDisplay(0); // clear screen

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(icpPin, INPUT);  // ICP pin (digital pin 8 on arduino) as input ---- was INPUT_PULLUP

  bit_time = 0;
  valid_tc_word = false;
  ones_bit_count = false;
  tc_sync = false;
  write_tc_out = false;
  drop_frame_flag = false;
  total_bits =  0;
  current_bit =  0;
  sync_count =  0;

  //Serial.println("Finished setup ");
  //delay (1000);

  TCCR1A = B00000000; // clear all
  TCCR1B = B11000010; // ICNC1 noise reduction + ICES1 start on rising edge + CS11 divide by 8
  TCCR1C = B00000000; // clear all
  TIMSK1 = B00100000; // ICIE1 enable the icp

  TCNT1 = 0; // clear timer1
  //http://arduino.on.kg/seven-segment-generator
  lc.setRow(0, 7, 0b00001111); //t
  lc.setRow(0, 6, 0b00000100); //i
  lc.setRow(0, 5, 0b00010101); //n
  lc.setRow(0, 4, 0b01001111); //E
  lc.setRow(0, 3, 0b00001101); //c
  lc.setRow(0, 2, 0b00011101); //o
  lc.setRow(0, 1, 0b00111101); //d
  lc.setRow(0, 0, 0b01001111); //E
  delay(1000);

  lc.setRow(0, 7, 0b00000000); //
  lc.setRow(0, 6, 0b00000101); //r
  lc.setRow(0, 5, 0b01001111); //E
  lc.setRow(0, 4, 0b01110111); //A
  lc.setRow(0, 3, 0b00111101); //d
  lc.setRow(0, 2, 0b01001111); //E
  lc.setRow(0, 1, 0b00000101); //r
  lc.setRow(0, 0, 0b00000000); //
  delay(1000);

  lc.setRow(0, 7, 0b01001110); //(
  lc.setRow(0, 6, 0b00001101); //c
  lc.setRow(0, 5, 0b01111000); //)
  lc.setRow(0, 4, 0b00000000); //
  lc.setRow(0, 3, 0b01101101); //2
  lc.setRow(0, 2, 0b01111110); //0
  lc.setRow(0, 1, 0b01101101); //2
  lc.setRow(0, 0, 0b01111110); //0
  delay(1000);


} //of setup()

void loop()
{
  if (write_tc_out)
  {
    write_tc_out = false;
    if (drop_frame_flag)
      Serial.print("TC-[df] ");
    else
      Serial.print("TC-[nd] ");
    for (int a = 0; a < 8; a++)
    {
      Serial.print(newtc[7 - a]);
    }
    Serial.print("\r\n");

    if (LED_MODE == 1) {
      for (int a = 0; a < 8; a++)
      {
        lc.setDigit(0, a, newtc[a], (a % 2 == 0) ? true : false);
      }
    }
    if (LED_MODE == 2) {
      lc.setDigit(0, 7, newtc[7], false);
      lc.setDigit(0, 6, newtc[6], false);
      lc.setChar (0, 5, '-', false);
      lc.setDigit(0, 4, newtc[5], false);
      lc.setDigit(0, 3, newtc[4], false);
      lc.setChar (0, 2, '-', false);
      lc.setDigit(0, 1, newtc[3], false);
      lc.setDigit(0, 0, newtc[2], (newtc[0] % 2 == 0) ? true : false);
    }
  }
} //of loop()
