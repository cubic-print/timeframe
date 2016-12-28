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
  
  
void setup() {
  pinMode(3, OUTPUT); //MAG: Timer 2B cycle output
  pinMode(10, OUTPUT); //LED: Timer 1B cycle output 
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22)| _BV(CS21)| _BV(CS20);
  OCR2A = 195;//80Hz
  OCR2B = 29;//15% duty cycle for magnet
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); 
  TCCR1B =  _BV(WGM13) | _BV(WGM12)  |  _BV(CS11);
  OCR1A = 24845;//80.5Hz
  OCR1B = 4969;//20% duty cycle for LED
  sei();
}

void loop() {

} //main loop

