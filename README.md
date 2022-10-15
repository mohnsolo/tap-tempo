# Tap Tempo

## Summary
Tap tempo control for PT2399-based delay effects pedals

## Background
This project integrates a tap tempo switch into existing PT2399-based delay effects, replacing the typical delay time potentiometer. Doing so allows a guitar player tap a footswitch to the desired tempo rather than bending down to turn a knob. The potentiometer is replaced by a rotary encoder knob for the purpose of making manual adjustments as desired. The code is written using vscode + platformio + Arduino framework.

## Dependencies
### Hardware
- Arduino UNO or similar board, or bare ATmega328P microcontroller ("MCU") with 16MHz external crystal + caps
- Delay effects pedal using a PT2399 echo/delay processor, such as a Rebote 2.5
- Digital potentiometer IC (This project uses an MCP41050, but others may work just as well with some code changes)
- Rotary encoder
### Libraries
- Arduino SPI library
- Bounce2 v2.53 by Thomas O Fredericks https://github.com/thomasfredericks/Bounce2
- Encoder v1.4.1 by Paul Stoffregen http://www.pjrc.com/teensy/td_libs_Encoder.html

## Functionality
The MCU performs the following tasks:
1. Measure the time between tap switch presses, take the average of the last 3 timings, apply division  
`timeInterval = (time1 + time2 + time3) / times / division`
2. Convert the averaged time to a resistance value, taken from the PT2399 datasheet: resistance **R** (kOhm) for a desired delay time **t** (ms) is approximately  
`R = t * 0.0885 - 2.7`
3. Set the digital potentiometer to this value via SPI
4. Check for updates to the encoder knob position and make updates to the resistance setpoint
5. If LFO pin is pulled low, modulate the resistance setpoint. *TODO: make LFO rate and depth controllable*

## Tempo Indicator
Shown in the [schematic](tap-tempo-schematic.pdf) is a tempo indication circuit using two 4040 binary counters to divide the clock signal out of pin 5 of the PT2399 by 684,000, the result of which corresponds to the actual delay time and is fed to an LED for visual feedback. This portion of the circuit is not my own, but is originally from [this post in the diystompboxes forum.](https://www.diystompboxes.com/smfforum/index.php?topic=50185.0) To reduce circuit size, this LED could be controlled by the MCU.
