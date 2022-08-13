#include <Bounce2.h>    // Bounce2 v2.53 by Thomas O Fredericks https://github.com/thomasfredericks/Bounce2
#include <SPI.h>        // Arduino SPI library
#include <Encoder.h>    // Encoder v1.4.1 by Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html

/*
 * fifo[3]
 * validTimes: number of valid time intervals in the fifo. The rest would be ignored
 *  when averaging. 
 */

const byte ss_pin = 10;    // set pin 10 as the slave select for the digital pot:
const byte tap_pin = 4;   // tap tempo button
const byte led_pin = 5;   // timing led
const byte div_pin = 6;   // divisions button
const byte lfo_pin = 7;   // lfo on/off switch
Bounce bTap = Bounce(); // Instantiate a Bounce object for tap button
Bounce bDiv = Bounce();   // bounce object for divisions button

unsigned long startTime;
unsigned long currentTime;
unsigned long timeInterval = 30;
unsigned long time1, time2, time3;
int times = 0;
unsigned long maxTime = 1200;
boolean timing = false;

float division = 1;
const byte div_array_max = 3;
float div_array[div_array_max] = {1.0, 1.5, 3.0};
byte div_index = 0;

boolean lfo_active = false;
byte lfo_amp = 5;     // amplitude of lfo
byte lfo_value = 0;
word lfo_speed_pot_set;   // 0-1023 analog value representing desired lfo speed
word lfo_speed_pot_new;
word lfo_max_time = 500;    // max rise and fall time in msn
word lfo_min_time = 50;    // min rise and fall time in ms
word lfo_set_time = 100;    // setting of rise and fall time in ms (full period is 2x this value)
word lfo_step_time = lfo_set_time / lfo_amp;       // time between lfo updates
unsigned long lfo_start_time;
boolean lfo_rising = 1;     // 1 = rising, 0 = falling

float Rset = 0.0;
float RcalH = 43.1;
float RcalL = 0.136;            // is this too small to matter?
float Roffset = 1.0 - RcalL;    // should this be addition?
float Rmax = RcalH;    // in kohms
byte potData = 127;
word potMax = 255;    // max value of digital pot
boolean newPotVal = false;      // true if ready to write new value after 4 taps

Encoder myEnc(2,3);
long oldPosition  = 0;
long newPosition;
long diff;

void setup() {
  bTap.attach(tap_pin,INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  bTap.interval(5); // Use a debounce interval of 25 milliseconds
  bDiv.attach(div_pin, INPUT_PULLUP);
  bDiv.interval(5);

  pinMode(lfo_pin,INPUT_PULLUP);  
  pinMode(led_pin,OUTPUT); // Setup the LED
  
  // set the ss_pin as an output:
  pinMode(ss_pin, OUTPUT);
  // initialize SPI:
  SPI.begin();
  //Serial.begin(9600);
  digitalPotWrite(0,potData);

  newPosition = myEnc.read();
}


void loop() {
  currentTime = millis();
  bTap.update(); // Update the Bounce instance
  bDiv.update();

  if (bDiv.fell())
  {
    if (++div_index >= div_array_max)
      div_index = 0;
    division = div_array[div_index];
  }
  
  if (bTap.fell() && !timing)       // First switch press
  {
    startTime = currentTime;
    timing = true;
    times = 0;
    time1 = 0; time2 = 0; time3 = 0;
    newPotVal = false;
    digitalWrite(led_pin, 1);
    bTap.update();   //ensures new debounce reading for second press
  }

  if (timing)
  {
    timeInterval = millis() - startTime;
    
    if (timeInterval > maxTime)     // don't count this time interval and end timing
    {
      timing = false;
      digitalWrite(led_pin, 0);
      if (newPotVal) digitalPotWrite(0,potData);
    }
    else if (bTap.fell())         // All other switch presses after the first
    {
      startTime = currentTime;
      if (times < 3) times++;
      time3 = time2;
      time2 = time1;
      time1 = timeInterval;
      if (times >= 3)
      {
        timeInterval = (time1 + time2 + time3) / times / division;
        if (timeInterval < 170)
          Rset = (float)timeInterval / 11.83 - 2.355;
        else
          Rset = (float)timeInterval / 11.03 - 3.468;
        Rset = Rset - Roffset;
        if (Rset < 0.0) Rset = 0;
        if (Rset > Rmax) Rset = Rmax;
        potData = (byte)(Rset / Rmax * potMax);
        newPotVal = true;
      }
    }
  }

  newPosition = myEnc.read();
  if (newPosition != oldPosition)
  {
    diff = newPosition - oldPosition;
    // check for over/under flow
    if (potData + diff > potMax)
    {
      potData = potMax;
    }
    else if (potData + diff < 0)
    {
      potData = 0;
    }
    else
    {
      potData += diff;
    }
    digitalPotWrite(0,potData);
    oldPosition = newPosition;
  }

  // LFO
  if (digitalRead(lfo_pin))
  {
    // Check if it's time to update lfo
    if (currentTime - lfo_start_time >= lfo_step_time)
    {
      /*lfo_speed_pot_new = analogRead(A1);
      if (lfo_speed_pot_set != lfo_speed_pot_new)
      {
        lfo_speed_pot_set = lfo_speed_pot_new;
    
        lfo_step_time = lfo_set_time / lfo_amp;
      }*/
      if (lfo_rising)
      {
        if (++lfo_value >= lfo_amp)
        {
          lfo_rising = 0;
        }
      }
      else
      {
        if (--lfo_value == 0)
        {
          lfo_rising = 1;
        }
      }
      lfo_start_time = currentTime;
      // Check for overflow
      if (potMax - potData >= lfo_value)
      {
        digitalPotWrite(0,potData + lfo_value);
      }
    }
  }

  // read the input on analog pin 0:
  //unsigned int sensorValue = analogRead(A0);
  //digitalPotWrite(0, (sensorValue >> 2));   // convert 10-bit sensorValue to 8-bit
  //delay(10);
}

void digitalPotWrite(byte channel, byte value) {
  byte command;
  command = 0x10 | (0x01 << (channel & 0x01));
  // take the SS pin low to select the chip:
  digitalWrite(ss_pin, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(command);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(ss_pin, HIGH);
}
