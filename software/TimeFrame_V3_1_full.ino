// TimeFrame V3.1 - simple version
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


//#define DEBUG
//uncomment to check serial monitor and see LED heartbeat
//Tactile Switch needs to be pressed longer in debug mode to change mode

//Base frequency and trimmer Ranges
#define BASE_FREQ 80.0 //80 in on the spot for many flowers. Feel free to play with this +/-5Hz
#define MIN_PHASE_SHIFT 0.1
#define MAX_PHASE_SHIFT 5.0
#define MIN_BRIGHTNESS 2 //if too low the movement will be only visible in darker rooms
#define MAX_BRIGHTNESS 10.0 //with high settings flickering will occur
  
  const int LED = 13; //on board LED
  const int SW = 6; //button for mode selection
  int mode_changed = 1;
  int mode = 1; //toggelt by SW button
  //mode 1 = normal slow motion mode (power on)
  //mode 2 = distorted reality mode
  //mode 3 = magnet off
  //mode 4 = completely off
  
  float phase_shift = 0.1; //eg. f=0.5 -> T=2 -> 2 seconds per slow motion cycle

  //Timer 2 for Magnet
  //Prescaler = 1024 = CS111 = 64us/tick
  //PIN 3
  float duty_mag = 15; //12 be carefull not overheat the magnet. better adjust force through magnet position
  float frequency_mag = BASE_FREQ;
  long time_mag = round(16000000/1024/frequency_mag); 

  //Timer 1 for LED 
  //Prescaler = 8 = CS010 = 0.5 us/tick
  //PIN 10
  float duty_led = 7;
  float frequency_led = frequency_mag+phase_shift; 
  long time_led = round(16000000/8/frequency_led);
  
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(LED, OUTPUT); //Heart Beat LED
  pinMode(SW, INPUT); //button pin
  pinMode(3, OUTPUT); //MAG: Timer 2B cycle output
  pinMode(10, OUTPUT); //LED: Timer 1B cycle output 

  #ifdef DEBUG
    pinMode(11, OUTPUT); //Timer 2A half frequency at 50% duty output for debugging halbe frequenz! 50% duty
    pinMode(9, OUTPUT); //Timer 1A half frequency at 50% duty output for debuggin
  #endif

  mag_on();
  OCR2A = round(time_mag); //Hierraus frequenz output compare registers
  OCR2B = round(duty_mag*time_mag/100L); //hierraus frequency output compare registers
  led_on();
  OCR1A = round(time_led); //Hierraus frequenz output compare registers
  OCR1B = round(duty_led*time_led/100L); //hierraus frequency output compare registers
  
  sei();
}

void loop() {
  //Read in trimmer settings
  phase_shift = -(MAX_PHASE_SHIFT-MIN_PHASE_SHIFT)/1023L*analogRead(A1)+MAX_PHASE_SHIFT; //Speed: 0.1 .. 5 Hz
  delay(3);
  duty_led = -(MAX_BRIGHTNESS-MIN_BRIGHTNESS)/1023L*analogRead(A0)+MAX_BRIGHTNESS;  //Brightness: duty_led 2..20
  frequency_led = frequency_mag*mode+phase_shift;
    
  if ((mode == 1) && (mode_changed == 1))
  {   
    frequency_mag = BASE_FREQ;
    mag_on();
    led_on();
    mode_changed = 0;
  }//mode = 1  
  if ((mode == 2) && (mode_changed == 1))
  {
    //frequency doubleing already done in main loop
    mode_changed = 0;
  }//mode = 2 
  if ((mode == 3) && (mode_changed == 1))
  { 
    mag_off(); //mode = 2
    mode_changed = 0;
  }//mode = 3 
  if ((mode == 4) && (mode_changed == 1))
  {
    led_off(); //mode = 4
    mode_changed = 0;
  }//mode = 4 

  time_mag = round(16000000L/1024L/frequency_mag); 
  time_led = round(16000000L/8L/frequency_led);

  OCR2A = round(time_mag); //to calculate frequency of output compare registers
  OCR2B = round(duty_mag*time_mag/100L); 
  OCR1A = round(time_led); 
  OCR1B = round(duty_led*time_led/100L);

  if (digitalRead(SW) == HIGH) //Read in switch
  {
    mode += 1;
    if (mode >= 5) mode = 1; //rotary menu
    delay(400); //400ms debounce
    mode_changed = 1;
  }

#ifdef DEBUG
 //Heatbeat on-board LED
  digitalWrite(LED, HIGH); // LED on
  delay(300);
  digitalWrite(LED, LOW); // LED off
  delay(300); 
  digitalWrite(LED, HIGH); // LED on
  delay(200);
  digitalWrite(LED, LOW); // LED off
  delay(1200); 
  //serial print current parameters
  Serial.print("Phase Shift: "); //speed of animation
  Serial.print(phase_shift);
  Serial.print("  Force: ");
  Serial.print(duty_mag);
  Serial.print("  Freq: ");
  Serial.print(frequency_mag);
  Serial.print("  Brightness: ");
  Serial.println(duty_led);
#endif
} //main loop

void mag_on() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22)| _BV(CS21)| _BV(CS20);
}

void mag_off() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A0) | _BV(COM2B1);
  TCCR2B = _BV(CS22)| _BV(CS21)| _BV(CS20); 
}

void led_on() {
TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); 
  TCCR1B =  _BV(WGM13) | _BV(WGM12)  |  _BV(CS11);
}

void led_off() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A0) | _BV(COM1B1); 
  TCCR1B =  _BV(CS11); 
}  
