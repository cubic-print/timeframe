// TimeFrame V3.0 - simple version
// Copyright (C) 2016 Cubc-Print

// get the latest source core here: http://www.github.com/cubic-print/timeframe
// video: http://youtu.be/LlGywKkifcI
// order your DIY kit here: http://www.cubic-print.com/TimeFrame

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    GNU General Public License terms: <http://www.gnu.org/licenses/>.

  const float freq = 80; //78.125; 
  
  float phase_shift = 0.5; //eg. f=0.5 -> T=2 -> 2 seconds per slow motion cycle

  //Timer 2 for Magnet
  //Prescaler = 1024 = CS111 = 64us/tick
  //PIN 3
  float duty_mag = 8; //12 be carefull not overheat the magnet. better adjustment through magnet position
  float frequency_mag = freq;
  long time_mag = round(16000000/1024/frequency_mag); 

  //Timer 1 for LED
  //Prescaler = 8 = CS010 = 0.5 us/tick
  //PIN 10
  float duty_led = 20;
  float frequency_led = frequency_mag+phase_shift;
  long time_led = round(16000000/8/frequency_led);
  
void setup() {
  pinMode(3, OUTPUT); //MAG: Timer 2B cycle output
  pinMode(10, OUTPUT); //LED: Timer 1B cycle output 
  mag_on();
  OCR2A = round(time_mag); //output compare register
  OCR2B = round(duty_mag*time_mag/100L); // output compare registers
  led_on();
  OCR1A = round(time_led); //output compare registers
  OCR1B = round(duty_led*time_led/100L); // output compare registers
  sei();
}

void loop() {

} //main loop

void mag_on() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22)| _BV(CS21)| _BV(CS20);
}

void led_on() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); 
  TCCR1B =  _BV(WGM13) | _BV(WGM12)  |  _BV(CS11);
}
