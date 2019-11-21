// TimeFrame v4.0
// Copyright (C) 2016 Cubic-Print
//
// Original source code here: http://www.github.com/cubic-print/timeframe
// Paul Hutchison modifications: https://github.com/paulh-rnd/timeframe
//
// This a complete re-write of the sketch with the following changes
// (Chris Satterlee, November 2019):
//
//    - More comments to help reader understand how it works
//    - More use of named constants for readability
//    - More use of functions for readability
//    - Better constant and variable names
//    - Unnecessary code removed
//    - On-board LED heartbeat is now always on, but code rewritten to
//      not interfere with timing
//    - Implementation of Paul Hutchison's improvements:
//        - Glowing LED for pushbutton (OPTIONAL)
//        - Frame starts in standby mode
//        - Additional analog input to control electromagnet duty
//          (OPTIONAL)
//        - Adjustments to ranges
//        - NOTE: Paul removed "Magnet Off" (LED only) mode. This code
//                brings it back (but it may be turned off with a
//                constant)
//    - NEW FEATURE: frequency control
//
// Note that this sketch is compatible with with both the original
// hardware design and with either or both of the hardware modifications
// added by Paul Hutchison (pushbutton LED and electromagnet strength
// knob.) However, the constants HW_HAS_PUSHBUTTON_LED and
// HW_HAS_MAG_STRENGTH_KNOB must be set appropriately below.
//
// The frequency control feature uses the same knob (potentiometer) as
// the LED brightness. The ability to vary the frequency allows the user
// to find the resonant frequency of the object(s). Frequency control
// mode is entered by "double clicking" the pushbutton when it is in the
// standby mode. The LED brightness value is captured on entry to the
// frequency control mode, and the knob is temporarily re-purposed to
// control the frequency. The speed control knob and the electromagnet
// strength knob work as normal while in frequency control
// mode. Frequency control mode is exited and normal slow motion mode is
// entered when the pushbutton is pressed. At this transition, the
// frequency value is captured and continues to be used while the frame
// is powered on. Additionally, the value is written to EEPROM. This
// value is read the next time the frame is powered on and is used until
// the user enters frequency control mode again to change it.
//
// In frequency control mode, the pushbutton LED blinks rapidly. On
// hardware that does not implement the pushbutton LED, the only way to
// tell that it is in frequency control mode is to go ahead and turn the
// knob that normally controls the brightness and see if it instead is
// controlling the frequency.
//
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
#include <EEPROM.h>

// Hardware feature constants
const bool HW_HAS_PUSHBUTTON_LED = true;
const bool HW_HAS_MAG_STRENGTH_KNOB = true;
// Software feature constants
const bool INCLUDE_MAGNET_OFF_MODE = true;
// Debug constants
const bool DEBUG_MODE = false;
const bool DEBUG_PRINT_OCRS = false; // Only if DEBUG_MODE
const bool DEBUG_PRINT_TARGET_VALUES = false; // Only if DEBUG_MODE
const uint16_t DEBUG_MS = 500; // Set to 0 for no delay
const uint16_t DEBUG_DECIMAL_PLACES = 2;
const uint32_t SERIAL_BAUD = 9600;
// ATmega328P constants
const float ANALOG_READ_MAX = 1023.0;
const float SYS_CLK_HZ = 16000000.0; // 16 Mhz
// Mode constants
const uint16_t POWER_ON = 0;
const uint16_t STANDBY = 1;
const uint16_t SLOW_MOTION = 2;
const uint16_t DISTORTED_REALITY = 3;
const uint16_t MAGNET_OFF = 4;
const uint16_t FREQ_CONTROL = 5;
// Pin constants
const uint8_t LED_DUTY_PIN = A0;   // Brightness
const uint8_t FREQ_DELTA_PIN = A1; // Slow motion speed
const uint8_t MAG_DUTY_PIN = A2;   // Magnet strength
const uint8_t FREQ_VALUE_PIN = A0; // Magnet frequency (shared with LED duty)
const uint16_t OC0A_PIN = 6; // Used for PUSHBUTTON_INPUT_PIN
const uint16_t OC0B_PIN = 5;
const uint16_t OC1A_PIN = 9;
const uint16_t OC1B_PIN = 10;
const uint16_t OC2A_PIN = 11;
const uint16_t OC2B_PIN = 3;
const uint16_t PUSHBUTTON_LED_PIN = OC0B_PIN; // Timer 0 (8-bit)
const uint16_t LED_PIN = OC1B_PIN;            // Timer 1 (16-bit)
const uint16_t MAG_PIN = OC2B_PIN;            // Timer 2 (8-bit)
const uint16_t PUSHBUTTON_INPUT_PIN = 6;
const uint16_t ONBOARD_LED_PIN = 13;
// Pushbutton LED constants
const float PUSHBUTTON_LED_STANDBY_PERIOD_MS = 3000.0;
const float PUSHBUTTON_LED_FREQ_CONTROL_PERIOD_MS = 200.0;
const float PUSHBUTTON_LED_MAX_DUTY = 100.0; // 255 = 100%
const float PUSHBUTTON_LED_OFF_PCT = 10.0;   // < 100
const float PUSHBUTTON_LED_DIM_RATIO = 4.0;
const uint16_t PUSHBUTTON_PRESSED = HIGH;
const uint16_t PUSHBUTTON_DEBOUNCE_MS = 100;
const uint16_t PUSHBUTTON_POLL_MS = 1;
const uint16_t PUSHBUTTON_DOUBLE_CLICK_MS = 300;
// Prescaler constants
const uint16_t TIMER1_PRESCALER = 8;    // LED PWM
const uint16_t TIMER2_PRESCALER = 1024; // Mag PWM
// EEPROM constants
const float EEPROM_VALID_ADDRESS = 0;
const float EEPROM_VALID_VALUE = 112233.01;  // Magic number @ addr 0
const float EEPROM_FREQ_ADDRESS = 4;
// Frequency and duty tuning constants
const float BASE_FREQ_HZ = 77.0;
const float FREQ_RANGE_HZ = 14.0; // Total range, centered around BASE_FREQ_HZ
const float MIN_FREQ_DELTA_HZ = 0.5; // Negative OK!
const float MAX_FREQ_DELTA_HZ = 2.0; // Negative OK, but must be > MIN
const float MIN_LED_DUTY_PCT = 0.0;
const float MAX_LED_DUTY_PCT = 20.0;
const float MIN_MAG_DUTY_PCT = 5.0;
const float MAX_MAG_DUTY_PCT = 25.0;
const float DEFAULT_MAG_DUTY_PCT = 7.0; // If no mag strength knob
// Heartbeat constants
const uint32_t HEARTBEAT_PERIOD_MS = 2000;
const uint32_t HEARTBEAT_T1_MS = 300; // On until T1
const uint32_t HEARTBEAT_T2_MS = 600; // Off until T2
const uint32_t HEARTBEAT_T3_MS = 800; // On until T3, then off
// Derived constants
const float PB_MAX_DUTY_DIVISOR = 2.0 - (PUSHBUTTON_LED_OFF_PCT / 100.0) * 2.0;
const float MIN_FREQ_HZ = BASE_FREQ_HZ - (FREQ_RANGE_HZ/2.0);
const float MAX_FREQ_HZ = BASE_FREQ_HZ + (FREQ_RANGE_HZ/2.0);
const float TIMER1_TICK_HZ = SYS_CLK_HZ / TIMER1_PRESCALER;
const float TIMER2_TICK_HZ = SYS_CLK_HZ / TIMER2_PRESCALER;
const float MIN_POSSIBLE_LED_HZ = TIMER1_TICK_HZ / (1L << 16);
const float MAX_POSSIBLE_LED_HZ = TIMER1_TICK_HZ / 2;
const float MIN_POSSIBLE_MAG_HZ = TIMER2_TICK_HZ / (1L << 8);
const float MAX_POSSIBLE_MAG_HZ = TIMER2_TICK_HZ / 2;

// Global variables
float mag_freq_hz;
float led_freq_hz;
float led_freq_delta_hz; // Slow motion speed (positive: LED freq is > magnet)
float led_duty_pct;      // Brightness
float mag_duty_pct;      // Electromagnet strength
uint32_t mode_start_time;
uint16_t mode = POWER_ON;
uint16_t prev_mode = POWER_ON;

void setup() {
  // Start serial
  Serial.begin(SERIAL_BAUD);

  // Set pin modes
  set_pin_modes();

  // Initialize mode timer
  capture_mode_start_time();

  // Set initial magnet frequency variable
  set_mag_freq();

  // POWER_ON mode -> STANDBY mode
  mode = STANDBY;
}

void loop() {
  // Per-mode actions
  switch (mode) {

  case STANDBY:
    if (mode_changed()) {
      mag_and_led_off();
    }
    update_pushbutton_led(); // Slow throb
    break;

  case SLOW_MOTION:
    if (mode_changed()) {
      update_pushbutton_led(); // solid on (dim)
      mag_and_led_on();
      if (prev_mode == FREQ_CONTROL)
        write_freq_to_eeprom();
    }
    break;

  case MAGNET_OFF:
    if (mode_changed())
      mag_off();
    break;

  case FREQ_CONTROL:
    update_pushbutton_led(); // Fast throb
    set_mag_freq();
    break;

  default:
    break;
  }

  // Get knob settings
  get_knob_settings();

  // Set LED frequency based on magnet frequency and knob
  set_led_freq();

  // Update Output Compare Registers
  update_ocrs();

  // Check pushbutton and change mode if required
  prev_mode = mode;
  mode = next_mode(mode);

  // Print debug info, if enabled
  debug_monitor();

  // Update onboard LED heartbeat value
  heartbeat();
}

void set_pin_modes() {
  // Outputs
  if (HW_HAS_PUSHBUTTON_LED)
    pinMode(PUSHBUTTON_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MAG_PIN, OUTPUT);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  if (DEBUG_MODE) {
    pinMode(OC1A_PIN, OUTPUT); // 1/2 LED frequency, 50% duty
    pinMode(OC2A_PIN, OUTPUT); // 1/2 magnet frequency, 50% duty
  }
  // Inputs
  pinMode(PUSHBUTTON_INPUT_PIN, INPUT);
  pinMode(LED_DUTY_PIN, INPUT);  // Aka FREQ_VALUE_PIN
  pinMode(FREQ_DELTA_PIN, INPUT);
  if (HW_HAS_MAG_STRENGTH_KNOB)
    pinMode(MAG_DUTY_PIN, INPUT);
}

void capture_mode_start_time() {
  mode_start_time = millis();
}

void set_mag_freq() {
  // Set the mag_freq_hz global variable. This depends on the mode. In
  // POWER_ON mode the mag_freq_hz value is read from EEPROM (if
  // valid). In FREQ_CONTROL mode, it is based on the frequency control
  // knob (which is the same as the brightness knob.) In other modes,
  // the value doesn't change.
  if (mode == POWER_ON) {
    mag_freq_hz = get_initial_mag_frequency();
  } else if (mode == FREQ_CONTROL) {
    mag_freq_hz = value_in_range(MIN_FREQ_HZ, MAX_FREQ_HZ,
                                 knob_value(FREQ_VALUE_PIN));
  }
  // Else keep current value

  // Constrain to limits
  mag_freq_hz = get_limited_mag_hz(mag_freq_hz);
}

void set_led_freq() {
  // The LED frequency is a delta from the magnet frequency. Since the
  // magnet PWM runs on the 8-bit Timer1, its actual frequency can be
  // different from the target frequency by a significant amount
  // relative to the delta, so we want to use the actual magnet
  // frequency. That is based on the OCR2A value. Note that since the
  // OCRs are updated after the call to this function in loop(), it will
  // not get properly updated until the subsequent loop() iteration.
  float actual_mag_freq_hz = calculate_actual_hz(2,TIMER2_PRESCALER);
  // Normal: LED freq > mag freq by delta (which can be negative)
  led_freq_hz = actual_mag_freq_hz + led_freq_delta_hz;
  if (mode == DISTORTED_REALITY)
    // Base frequency doubled
    led_freq_hz = (actual_mag_freq_hz * 2) + led_freq_delta_hz;

  // Constrain to limits
  led_freq_hz = get_limited_led_hz(led_freq_hz);
}

float get_initial_mag_frequency() {
  // Read the EEPROM to get the initial frequency. If the EEPROM has not
  // been programmed with the frequency, use BASE_FREQ_HZ.
  uint16_t eeprom_valid_count;
  float eeprom_value;
  // Check that address 0 has "magic" value
  EEPROM.get(EEPROM_VALID_ADDRESS, eeprom_value);
  if (eeprom_value == EEPROM_VALID_VALUE) {
    EEPROM.get(EEPROM_FREQ_ADDRESS, eeprom_value);
    if (DEBUG_MODE) {
      Serial.print("Initial magnet frequency (from EEPROM): ");
      Serial.println(eeprom_value);
    }
    return (eeprom_value);
  } else {
    // If EEPROM is not programmed, use BASE_FREQ_HZ
    Serial.print("Initial magnet frequency (BASE_FREQ_HZ): ");
    Serial.println(BASE_FREQ_HZ);
    return (BASE_FREQ_HZ);
  }
}

void write_freq_to_eeprom() {
  // Write magic number to address 0 and magnet frequency to address 4
  EEPROM.put(EEPROM_VALID_ADDRESS, EEPROM_VALID_VALUE);
  EEPROM.put(EEPROM_FREQ_ADDRESS, mag_freq_hz);
  if (DEBUG_MODE) {
    Serial.print("Saved magnet frequency to EEPROM: ");
    Serial.println(mag_freq_hz);
  }
}

float knob_value(uint8_t pin) {
  // Returns a value between 0.0 (knob turned max counterclockwise) and
  // 1.0 (knob turned max clockwise).
  //
  // The potentiometer resistance is zero when the knob is turned all
  // the way counterclockwise. This results in a voltage of +5V at the
  // analog input pin and analogRead() returns a value of 1023. The
  // potentiometer resistance is at its max when the knob is turned all
  // the way clockwise. This results in a voltage of zero at the analog
  // input pin and analogRead() returns a value of 0. This is opposite
  // from what we want.
  //
  // This function flips the sense, and scales the value so that the
  // value returned increases from zero to one as the knob is turned
  // clockwise.
  return ((ANALOG_READ_MAX - analogRead(pin)) / ANALOG_READ_MAX);
}

float value_in_range(float min_value, float max_value, float knob_value) {
  // This function uses the knob value and translates it to a value
  // between the specified minimum and maximum values.
  return (min_value + ((max_value - min_value) * knob_value));
}

void get_knob_settings() {
  set_led_freq_delta();
  set_led_duty_pct();
  set_mag_duty_pct();
}

void set_led_freq_delta() {
  // Set the led_freq_delta_hz global variable based on the slow motion
  // speed knob position
  led_freq_delta_hz = value_in_range(MIN_FREQ_DELTA_HZ, MAX_FREQ_DELTA_HZ,
                                     knob_value(FREQ_DELTA_PIN));
}

void set_led_duty_pct() {
  // In frequency control mode, the LED brightness knob is "borrowed"
  // for adjusting the frequency, so the current led_duty_pct value is
  // maintained. However, once slow motion mode is re-entered, the LED
  // duty will be set based on where the knob was left from the
  // frequency adjustment. The user will just have to readjust the
  // brightness at that point.
  if (mode != FREQ_CONTROL) {
    // Set the led_duty_pct global variable based on the brightness knob
    // position
    led_duty_pct = value_in_range(MIN_LED_DUTY_PCT, MAX_LED_DUTY_PCT,
                                  knob_value(LED_DUTY_PIN));
  }
}

void set_mag_duty_pct() {
  if (HW_HAS_MAG_STRENGTH_KNOB) {
    // Set the mag_duty_pct global variable based on the magnet strength
    // knob position
    mag_duty_pct = value_in_range(MIN_MAG_DUTY_PCT, MAX_MAG_DUTY_PCT,
                                  knob_value(MAG_DUTY_PIN));
  } else {
    // No knob: use default
    mag_duty_pct = DEFAULT_MAG_DUTY_PCT;
  }
}

uint16_t next_mode(uint16_t current_mode) {
  uint16_t next_mode = STANDBY;
  uint32_t current_mode_time;

  // POWER_ON mode immediately transitions to STANDBY
  if (current_mode == POWER_ON)
    return STANDBY;

  // Get current mode time. Note that this is the time between the
  // previous pushbutton -release- and the current pushbutton -press-.
  current_mode_time = get_current_mode_time();

  // If button is not pressed or if this is a release bounce, keep
  // current mode
  if (!pushbutton_pressed() || (current_mode_time < PUSHBUTTON_DEBOUNCE_MS))
    return current_mode;

  // Button is pressed. Wait until it is released
  wait_for_pushbutton_release();

  // Determine next mode
  switch (current_mode) {
  case STANDBY:
    next_mode = SLOW_MOTION;
    break;
  case SLOW_MOTION:
    next_mode = DISTORTED_REALITY; // Normal case
    if (current_mode_time < PUSHBUTTON_DOUBLE_CLICK_MS)
      // If we've been in slow motion mode for a very short amount of
      // time, this is the "double click" that overrides the normal case
      // and puts us into frequency control mode.
      next_mode = FREQ_CONTROL;
    break;
  case DISTORTED_REALITY:
    next_mode = STANDBY;
    if (INCLUDE_MAGNET_OFF_MODE)
      next_mode = MAGNET_OFF;
    break;
  case MAGNET_OFF:
    next_mode = STANDBY;
    break;
  case FREQ_CONTROL:
    next_mode = SLOW_MOTION;
    break;
  }
  capture_mode_start_time();
  return next_mode;
}

bool mode_changed() {
  return (mode != prev_mode);
}

bool pushbutton_pressed() {
  if (digitalRead(PUSHBUTTON_INPUT_PIN) == PUSHBUTTON_PRESSED)
    return true;
  // Otherwise, it's not pressed - don't read it again!
  return false;
}

void wait_for_pushbutton_release() {
  delay(PUSHBUTTON_DEBOUNCE_MS);
  while (pushbutton_pressed())
    delay(PUSHBUTTON_POLL_MS);
}

uint32_t get_current_mode_time() {
  uint32_t current_time = millis();
  if (mode_start_time > current_time) {
    // Timer wrapped (happens once every 50 days). No need to get fancy,
    // just return current time.
    return current_time;
  }
  // Normal case
  return (current_time - mode_start_time);
}

void update_pushbutton_led() {
  uint16_t duty;
  if (!HW_HAS_PUSHBUTTON_LED)
    return;
  switch (mode) {
  case STANDBY:
    duty = get_pushbutton_led_duty(PUSHBUTTON_LED_STANDBY_PERIOD_MS);
    break;
  case SLOW_MOTION:
  case DISTORTED_REALITY:
    duty = PUSHBUTTON_LED_MAX_DUTY / PUSHBUTTON_LED_DIM_RATIO;
    break;
  case FREQ_CONTROL:
    duty = get_pushbutton_led_duty(PUSHBUTTON_LED_FREQ_CONTROL_PERIOD_MS);
    break;
  }
  analogWrite(PUSHBUTTON_LED_PIN, duty);
}

uint16_t get_pushbutton_led_duty(float period_ms) {
  // In STANDBY mode the pushbutton LED "throbs" (for lack of a better
  // word). This is accomplished by cyclically modifying the duty cycle
  // of the PWM pin that drives it.
  // 
  // The pushbutton LED uses "simple PWM", i.e. analogWrite(). The
  // "value" parameter for analogWrite() determines the duty cycle and
  // is an integer between 0 (always off) to 255 (always on).
  //
  // This function is passed a period (interval between throbs) in
  // milliseconds. It uses that, along with the current time in
  // milliseconds to determine the current duty cycle value for
  // analogWrite(). The throbbing is sinusoidal. Complicating the math a
  // bit is support for having the LED be off for some percentage of
  // each period (PUSHBUTTON_LED_OFF_PCT).
  //
  // This function is also used for the much faster throbbing in
  // FREQ_CONTROL mode. Simple blinking would have been fine for that,
  // but it was easier to just use this same function with a shorter
  // period.
  float radians = 2.0 * PI * millis() / period_ms;
  int16_t led_duty = ((PUSHBUTTON_LED_MAX_DUTY / PB_MAX_DUTY_DIVISOR) *
                      ((PB_MAX_DUTY_DIVISOR - 1.0) + sin(radians)));
  led_duty = (led_duty < 0) ? 0 : led_duty; // Off when negative
  return led_duty;
}

void update_ocrs() {
  // The ATmega328P Output Compare Registers (OCRs) determine the
  // frequency and duty cycle of the PWM signal. With the TCCRs
  // programmed as they are by led_on() and mag_on(), this works as
  // follows:
  //
  // OCRnA is programmed with the TOP value and OCRnB is programmed with
  // the compare value.
  //
  // The TOP value of the timer is the value at which it will wrap back
  // to BOTTOM (i.e. zero). This determines the frequency (but not duty
  // cycle) of the PWM output; the larger the TOP value, the slower the
  // PWM output frequency. The timer runs at the chip clock frequency
  // (16 Mhz) divided by the prescaler (programmed in the TCCRnB
  // register). Setting the TOP value to 1 results in a frequency that
  // is equal to half the timer frequency (e.g. 1 MHz for a prescaler of
  // 8, 7.8125 kHz fpr a prescaler of 1024). The maximum TOP value for
  // the 8-bit timers (Timer0 and Timer2) is 255, and the maximum TOP
  // value for the 16-bit Timer1 is 65535.
  //
  // The PWM output is HIGH when the timer equals 0 and continues to be
  // HIGH up to and including the cycle when the timer equals the
  // compare value. The PWM output is low when the timer value is
  // greater than the compare value. This determines the duty cycle.
  update_led_ocrs();
  update_mag_ocrs();
}
void update_led_ocrs() {
  uint32_t top_value;
  uint32_t compare_value;
  top_value = get_top_value(led_freq_hz, TIMER1_PRESCALER);
  compare_value = get_compare_value(led_duty_pct, top_value);
  OCR1A = top_value;
  OCR1B = compare_value;
}

void update_mag_ocrs() {
  uint32_t top_value;
  uint32_t compare_value;
  top_value = get_top_value(mag_freq_hz, TIMER2_PRESCALER);
  compare_value = get_compare_value(mag_duty_pct, top_value);
  OCR2A = top_value;
  OCR2B = compare_value;
}

uint32_t get_top_value(float hz_tgt, uint16_t prescaler) {
  // Get the TOP value (OCRnA) that will achieve a PWM frequency closest
  // to the target
  float top_value_float;
  uint32_t top_value;
  top_value_float = ((SYS_CLK_HZ / prescaler) / hz_tgt) - 1.0;
  top_value = top_value_float + 0.5; // rounds to integer
  return top_value;
}

uint32_t get_compare_value(float duty_pct_tgt, uint32_t top_value) {
  // Get the compare value (OCRnB) that will achieve a duty cycle
  // closest to the target
  uint32_t compare_value_float;
  uint32_t compare_value;
  compare_value_float = ((duty_pct_tgt / 100) * (top_value + 1)) - 1.0;
  compare_value = compare_value_float + 0.5; // rounds to integer
  return compare_value;
}

float calculate_actual_hz(uint16_t timer, uint16_t prescaler) {
  uint32_t timer_top_value;
  switch (timer) {
  case 0:
    timer_top_value = OCR0A;
    break;
  case 1:
    timer_top_value = OCR1A;
    break;
  case 2:
    timer_top_value = OCR2A;
    break;
  }
  return (SYS_CLK_HZ / prescaler / (timer_top_value + 1));
}

float calculate_actual_duty_pct(uint16_t timer) {
  uint32_t timer_top_value;
  uint32_t timer_compare_value;
  switch (timer) {
  case 0:
    timer_top_value = OCR0A;
    timer_compare_value = OCR0B;
    break;
  case 1:
    timer_top_value = OCR1A;
    timer_compare_value = OCR1B;
    break;
  case 2:
    timer_top_value = OCR2A;
    timer_compare_value = OCR2B;
    break;
  }
  return ((float(timer_compare_value) + 1) / (timer_top_value + 1) * 100);
}

float get_limited_mag_hz(float mag_freq_hz) {
  if (mag_freq_hz < MIN_POSSIBLE_MAG_HZ) {
    if (DEBUG_MODE) {
      Serial.print("Magnet frequency ");
      Serial.print(mag_freq_hz);
      Serial.print(" Hz is too low, using ");
      Serial.println(MIN_POSSIBLE_MAG_HZ);
    }
    return MIN_POSSIBLE_MAG_HZ;
  }
  if (mag_freq_hz > MAX_POSSIBLE_MAG_HZ) {
    if (DEBUG_MODE) {
      Serial.print("Magnet frequency ");
      Serial.print(mag_freq_hz);
      Serial.print(" Hz is too high, using ");
      Serial.println(MAX_POSSIBLE_MAG_HZ);
    }
    return MAX_POSSIBLE_MAG_HZ;
  }
  return mag_freq_hz;
}

float get_limited_led_hz(float led_freq_hz) {
  if (led_freq_hz < MIN_POSSIBLE_LED_HZ) {
    if (DEBUG_MODE) {
      Serial.print("LED frequency ");
      Serial.print(led_freq_hz);
      Serial.print(" Hz is too low, using ");
      Serial.println(MIN_POSSIBLE_LED_HZ);
    }
    return MIN_POSSIBLE_LED_HZ;
  }
  if (led_freq_hz > MAX_POSSIBLE_LED_HZ) {
    if (DEBUG_MODE) {
      Serial.print("LED frequency ");
      Serial.print(led_freq_hz);
      Serial.print(" Hz is too high, using ");
      Serial.println(MAX_POSSIBLE_LED_HZ);
    }
    return MAX_POSSIBLE_LED_HZ;
  }
  return led_freq_hz;
}

void mag_and_led_on() {
  mag_on();
  led_on();
}

void mag_and_led_off() {
  mag_off();
  led_off();
}

void led_on() {
  // Write TCCR1A and TCCR1B registers with:
  //  COM1A = b01 (OC1A pin toggles on compare match)
  //  COM1B = b10 (OC1B pin cleared on compare match and set at 0)
  //  WGM = b1111 (waveform generation mode 15)
  //  CS = b010 (prescaler = 8)
  // ATmega328P Datasheet:
  //    Table 16-2 (COM), Table 16-4 (WGM), Table 16-5 (CS)
  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
}

void led_off() {
  // WGM = b0000 (Waveform generation mode 0)
  TCCR1A = _BV(COM1A0) | _BV(COM1B1);
  TCCR1B = _BV(CS11);
}

void mag_on() {
  // Write TCCR2A and TCCR2B registers with:
  //  COM2A = b01 (OC2A pin toggles on compare match)
  //  COM2B = b10 (OC2B pin cleared on compare match and set at 0)
  //  WGM = b111 (waveform generation mode 7)
  //  CS = b111 (prescaler = 1024)
  // ATmega328P Datasheet:
  //    Tables 18-3,18-6 (COM), Table 18-8 (WGM), Table 18-9 (CS)
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21) | _BV(CS20);
}

void mag_off() {
  // WGM = b000 (Waveform generation mode 0)
  TCCR2A = _BV(COM2A0) | _BV(COM2B1);
  TCCR2B = _BV(CS22) | _BV(CS21)| _BV(CS20);
}

void debug_monitor() {
  // Print the actual frequency, duty and delta values (based on
  // OCRs). Optionally print the OCR values themselves and the target
  // frequency, duty and delta values.
  if (DEBUG_MODE) {
    if (millis() % (DEBUG_MS + 1) != 0)
      // Rate limit debug to once per DEBUG_MS milliseconds.
      return;
    float actual_led_freq_hz = calculate_actual_hz(1,TIMER1_PRESCALER);
    float actual_mag_freq_hz = calculate_actual_hz(2,TIMER2_PRESCALER);
    float actual_freq_delta_hz = actual_led_freq_hz - actual_mag_freq_hz;
    print_mode();
    Serial.print(" Mode,");
    Serial.print(" LED: ");
    Serial.print(actual_led_freq_hz, DEBUG_DECIMAL_PLACES);
    Serial.print(" Hz, ");
    Serial.print(calculate_actual_duty_pct(1), DEBUG_DECIMAL_PLACES);
    Serial.print("% duty");
    Serial.print("  Mag: ");
    Serial.print(actual_mag_freq_hz, DEBUG_DECIMAL_PLACES);
    Serial.print(" Hz, ");
    Serial.print(calculate_actual_duty_pct(2), DEBUG_DECIMAL_PLACES);
    Serial.print("% duty");
    Serial.print("  Delta: ");
    Serial.print(actual_freq_delta_hz, DEBUG_DECIMAL_PLACES);
    Serial.print(" Hz");
    if (DEBUG_PRINT_OCRS) {
      Serial.print("  OCR1A: ");
      Serial.print(OCR1A);
      Serial.print("  OCR1B: ");
      Serial.print(OCR1B);
      Serial.print("  OCR2A: ");
      Serial.print(OCR2A);
      Serial.print("  OCR2B: ");
      Serial.print(OCR2B);
    }
    Serial.println("");
    if (DEBUG_PRINT_TARGET_VALUES) {
      print_target_values();
    }
  }
}

void print_target_values() {
  // These are the "ideal" values for frequency, duty and delta before
  // the rounding effects of the 8-bit and 16-bit OCRs.
  Serial.print("              Targets: ");
  Serial.print(" LED: ");
  Serial.print(led_freq_hz, DEBUG_DECIMAL_PLACES);
  Serial.print(" Hz, ");
  Serial.print(led_duty_pct, DEBUG_DECIMAL_PLACES);
  Serial.print("% duty");
  Serial.print("  Mag: ");
  Serial.print(mag_freq_hz, DEBUG_DECIMAL_PLACES);
  Serial.print(" Hz, ");
  Serial.print(mag_duty_pct, DEBUG_DECIMAL_PLACES);
  Serial.print("% duty");
  Serial.print("  Delta: ");
  Serial.print(led_freq_delta_hz, DEBUG_DECIMAL_PLACES);
  Serial.println(" Hz");
}

void print_mode() {
  // Prints mode name (no trailing whitespace or carriage return)
  if (mode == POWER_ON)
    Serial.print("         POWER_ON");
  else if (mode == STANDBY)
    Serial.print("          STANDBY");
  else if (mode == SLOW_MOTION)
    Serial.print("      SLOW_MOTION");
  else if (mode == DISTORTED_REALITY)
    Serial.print("DISTORTED_REALITY");
  else if (mode == MAGNET_OFF)
    Serial.print("       MAGNET_OFF");
  else if (mode == FREQ_CONTROL)
    Serial.print("     FREQ_CONTROL");
  else
    Serial.print("          UNKNOWN");
}

void heartbeat() {
  // Non-blocking, i.e. no calls to delay()
  uint32_t ms_in_period = millis() % HEARTBEAT_PERIOD_MS;
  if (ms_in_period < HEARTBEAT_T1_MS) {
    // HIGH until T1
    digitalWrite(ONBOARD_LED_PIN, HIGH); // Thump
  } else if (ms_in_period < HEARTBEAT_T2_MS) {
    // LOW until T2
    digitalWrite(ONBOARD_LED_PIN, LOW);
  } else if (ms_in_period < HEARTBEAT_T3_MS) {
    // HIGH until T3
    digitalWrite(ONBOARD_LED_PIN, HIGH); // Thump
  } else {
    // LOW the rest of the period
    digitalWrite(ONBOARD_LED_PIN, LOW);
  }
}
