#include <Arduino.h>
#include <Bounce2.h>    // Bounce2 v2.53 by Thomas O Fredericks https://github.com/thomasfredericks/Bounce2
#include <SPI.h>        // Arduino SPI library
#include <Encoder.h>    // Encoder v1.4.1 by Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html

//#define TIMING_LED   // uses pin A1 as an indicator that tap timing is underway
//#define CALIB_SERIAL_OUT  // outputs result of calibration to serial
#define NO_CALIB      // bypasses calibration on startup and sets Rmax to hard-coded value, RcalH

// Function declarations
void digitalPotWrite(byte channel, byte value);

const byte ss_pin = 10;    // set pin 10 as the slave select for the digital pot:
const byte tap_pin = 4;   // tap tempo button
#ifdef TIMING_LED
const byte timing_led_pin = A1;   // timing led
#endif
const byte div_pin = 6;   // divisions button
const byte lfo_pin = 7;   // lfo on/off switch
const byte enca_pin = 2;  // Interrupt pin on Arduino Uno
const byte encb_pin = 3;  // Interrupt pin on Arduino Uno
const byte div_led1_pin = 9; 
const byte div_led2_pin = 8;
const byte div_led3_pin = 5;
const byte fck_div_pin = 12;  // output of PT2399 pin 5 after division of 684000
const byte rate_led_pin = A0;
const byte rate_led_bit = 0;

Bounce bTap = Bounce(); // Instantiate a Bounce object for tap button
Bounce bDiv = Bounce();   // bounce object for divisions button

unsigned long startTime;
unsigned long currentTime;
unsigned long timeInterval = 30;
const byte num_valid_times = 3;     // number of taps - 1
unsigned long measured_times[num_valid_times] = {0};
byte times = 0;
unsigned long maxTime = 750; 
boolean timing = false;
boolean rate_led_state = false;

float division;
const byte div_array_max = 3;
float div_array[div_array_max] = {1.0, 1.5, 3.0};
byte div_led_array[div_array_max] = {div_led1_pin, div_led2_pin, div_led3_pin};
byte div_index = 0;

boolean lfo_active = false;
byte lfo_amp = 5;     // amplitude of lfo
byte lfo_value = 0;
word lfo_speed_pot_set;   // 0-1023 analog value representing desired lfo speed
word lfo_speed_pot_new;
word lfo_max_time = 500;    // max rise and fall time in ms
word lfo_min_time = 50;    // min rise and fall time in ms
word lfo_set_time = 100;    // setting of rise and fall time in ms (full period is 2x this value)
word lfo_step_time = lfo_set_time / lfo_amp;       // time between lfo updates
unsigned long lfo_start_time;
boolean lfo_rising = 1;     // 1 = rising, 0 = falling

// PT2399 approximate resistance R (kOhm) for delay time t (ms): 
// R = t * 0.0885 - 2.7
// All resistor values in kOhm
float slope = 0.0885;
float yint = -2.7;

float Rset = 0.0;
float RcalH = 43.1;   // measured from actual MCP41050 (calibration routine is a far better way than this)
float Roffset = 1.0;
float Rmax = 50; 
byte potData = 127;
word potMax = 255;    // max value of digital pot
boolean newPotVal = false;      // true if ready to write new value after 4 taps
const byte pot_ch = 0; 

Encoder myEnc(enca_pin,encb_pin);
long oldPosition  = 0;
long newPosition;
long diff;

void setup() {
  bTap.attach(tap_pin,INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  bTap.interval(5); // Use a debounce interval of 25 milliseconds
  bDiv.attach(div_pin, INPUT_PULLUP);
  bDiv.interval(5);

  pinMode(lfo_pin,INPUT_PULLUP);  
#ifdef TIMING_LED
  pinMode(timing_led_pin,OUTPUT); // Setup the LED
#endif
  pinMode(div_led1_pin, OUTPUT);
  pinMode(div_led2_pin, OUTPUT);
  pinMode(div_led3_pin, OUTPUT);
  pinMode(rate_led_pin, OUTPUT);
  digitalWrite(div_led1_pin, 0);
  digitalWrite(div_led2_pin, 0);
  digitalWrite(div_led3_pin, 0);
  digitalWrite(rate_led_pin, 0);
  
  division = div_array[div_index];  // initialize division
  digitalWrite(div_led_array[div_index], 1);

  // set the ss_pin as an output:
  pinMode(ss_pin, OUTPUT);
  // initialize SPI:
  SPI.begin();

#ifdef NO_CALIB
  Rmax = RcalH;
#else
  // Calibrate digital pot
  pinMode(fck_div_pin, INPUT);
  digitalPotWrite(pot_ch,255);
  delay(500);
  while(digitalRead(fck_div_pin) == 0);   // wait for pin to be low
  while(digitalRead(fck_div_pin) == 1);   // rising edge
  startTime = millis();
  while(digitalRead(fck_div_pin) == 0);   // wait for pin to be low
  while(digitalRead(fck_div_pin) == 1);   // rising edge
  currentTime = millis();
  timeInterval = currentTime - startTime;
  Rmax = (float)(timeInterval) * slope + yint - Roffset;
#endif

#ifdef CALIB_SERIAL_OUT
  Serial.begin(9600);
  Serial.println(timeInterval);
  Serial.println(Rmax, 3);
  Serial.end();
#endif

  digitalPotWrite(pot_ch,potData);
  newPosition = myEnc.read();
}


void loop() {
  PORTC ^= 1 << rate_led_bit;   // TESTING: toggle rate led output pin for viewing on a scope
  currentTime = millis();
  bTap.update(); // Update the Bounce instance
  bDiv.update();

  if (bDiv.fell())
  {
    digitalWrite(div_led_array[div_index], 0);    // turn off LED of old division (before we increment div_index)
    if (++div_index >= div_array_max)
      div_index = 0;
    division = div_array[div_index];
    digitalWrite(div_led_array[div_index], 1);    // turn on LED of new division
  }
  
  if (bTap.fell() && !timing)       // First switch press
  {
    startTime = currentTime;
    timing = true;
    times = 0;
    for (int i = 0; i < num_valid_times; i++)
    {
      measured_times[i] = 0;
    }
    newPotVal = false;
#ifdef TIMING_LED
    digitalWrite(timing_led_pin, 1);
#endif
    bTap.update();   //ensures new debounce reading for second press
  }

  if (timing)
  {
    timeInterval = millis() - startTime;
    
    if (timeInterval > maxTime)     // don't count this time interval and end timing
    {
      timing = false;
#ifdef TIMING_LED
      digitalWrite(timing_led_pin, 0);
#endif
      if (newPotVal) digitalPotWrite(pot_ch,potData);
    }
    else if (bTap.fell())         // All other switch presses after the first
    {
      startTime = currentTime;
      if (times < num_valid_times) times++;
      for (int i = num_valid_times - 1; i > 0; i--)
      { 
        measured_times[i] = measured_times[i-1];
      } 
      measured_times[0] = timeInterval;
      if (times >= num_valid_times)
      {
        timeInterval = 0;
        for (int i = 0; i < num_valid_times; i++)
        {
          timeInterval += measured_times[i];
        }
        timeInterval = timeInterval / times / division;
        Rset = (float)timeInterval * slope + yint;
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
    digitalPotWrite(pot_ch,potData);
    oldPosition = newPosition;
  }

  // LFO
  if (digitalRead(lfo_pin))
  {
    // Check if it's time to update lfo
    if (currentTime - lfo_start_time >= lfo_step_time)
    {
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
        digitalPotWrite(pot_ch,potData + lfo_value);
      }
    }
  }
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
