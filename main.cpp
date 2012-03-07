// Copyright (c) 2011 Shaun Crampton.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdlib.h>
#include <stdarg.h>
#include "WProgram.h"
#include "cppboilerplate.h"
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <EEPROM.h>

#define TARGET_LOOP_HZ (500)
#define TICKS_PER_LOOP (F_CPU / TARGET_LOOP_HZ)
#define TICK_SECONDS (1.0 / TARGET_LOOP_HZ)

// Arduino has a 10-bit ADC, giving a range of 0-1023.  0 represents GND, 1023
// represents Vcc - 1 LSB.  Hence, 512 is the midpoint.
#define ADC_RANGE (1024)
#define ADC_HALF_RANGE (512)

// Calculate the scaling factor from gyro ADC reading to radians.  The gyro is
// ratiometric with Vs but it doesn't use the full range.
#define RADIANS_PER_DEGREE 0.0174532925
#define V_PER_ADC_UNIT (5.0 / ADC_RANGE)
#define GYRO_MAX_DEG_PER_SEC (150.0)      // From gyro datasheet
#define GYRO_V_PER_DEG_PER_SEC (0.01125)  // Calibrated value from datasheet.
#define GYRO_DEG_PER_ADC_UNIT (V_PER_ADC_UNIT / GYRO_V_PER_DEG_PER_SEC)
#define GYRO_RAD_PER_ADC_UNIT (GYRO_DEG_PER_ADC_UNIT * RADIANS_PER_DEGREE)

// Calculate the scaling factor from ADC readings to g.
//
// The accelerometer is ratiometric with Vs but it doesn't use the full range
// from GND to Vs.  At Vs = 5V, it reads about 1V per g.
#define ACCEL_V_PER_G (1.0)
#define ACCEL_G_PER_ADC_UNIT (V_PER_ADC_UNIT / ACCEL_V_PER_G)

// EEPROM addresses
#define GYRO_OFFSET_ADDR 0
#define GYRO_OFFSET_SIZE sizeof(float)
#define ACCEL_FACT_ADDR (GYRO_OFFSET_ADDR + GYRO_OFFSET_SIZE)
#define ACCEL_FACT_SIZE sizeof(float)

// Pin definitions
const int led_pin = 7;
const int pwm_a = 3;   // PWM control for motor outputs 1 and 2 is on digital pin 3
const int pwm_b = 11;  // PWM control for motor outputs 3 and 4 is on digital pin 11
const int dir_a = 12;  // direction control for motor outputs 1 and 2 is on digital pin 12
const int dir_b = 13;  // direction control for motor outputs 3 and 4 is on digital pin 13
const int x_pin = A0;
const int y_pin = A1;
const int gyro_pin = A2;
const int switch_pin = 8;  // Pin connected to our active-low push switch.  Assumed to be
                           // PORTB 0 below when enabling the internal pullup.

// Default gyro offset used if no calibration has been done.  Unique to each
// gyro.
const float GYRO_OFFSET = 3.982 * GYRO_RAD_PER_ADC_UNIT;

// Allowances for mechanical differences in motors
#define MOTOR_A_FACTOR 1
#define MOTOR_B_FACTOR 1

/**
 * Union used to convert from float to bytes and back again.
 */
union floatbytes
{
  char bytes[sizeof(float)];
  float val;
} floatbytes;

static void write_float_to_eeprom(int addr, float f)
{
  floatbytes.val = f;
  for (unsigned int i = 0; i < sizeof(float); i++)
  {
    EEPROM.write(addr + i, floatbytes.bytes[i]);
  }
}

static float read_float_from_eeprom(int addr)
{
  for (unsigned int i = 0; i < sizeof(float); i++)
  {
    floatbytes.bytes[i] = EEPROM.read(addr + i);
  }
  return floatbytes.val;
}

// Filter orders.
#define NZEROS 2
#define NPOLES 2

/**
 * Low pass filter. Created via
 * http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
 */
static float filterx(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / 1.058546241e+03;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                          + ( -0.9149758348 * yv[0]) + (  1.9111970674 * yv[1]);
  return yv[2];
}
static float filtery(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / 1.058546241e+03;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                          + ( -0.9149758348 * yv[0]) + (  1.9111970674 * yv[1]);
  return yv[2];
}

void myprintf(const char *fmt, ... ){
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  Serial.print(tmp);
}

// Values read from the trim pots.
static int d_tilt_pot = 512;
static int x_offset_pot = 512;
static int gyro_offset_pot = 512;
// Raw 0-ADC_RANGE readings from the sensors.
static int gyro_reading = 0;
static int x_reading = 0;
static int y_reading = 0;
// Gyro rate converted to rads per sec.
static float gyro_rads_per_sec;
// Unfiltered accelerometer values in g.
static float y_gs;
static float x_gs;
// Filtered accelerometer readings in g.
static float x_filt_gs = 0;
static float y_filt_gs = 0;
// Best estimate of the curent tilt.
static float tilt_rads_estimate = 0;
// Integral of tilt.
static float tilt_int_rads = 0;
// Calculated speed.
static float speed = 0;
// Individual speeds applied to each motor.
static long int motor_a_speed = 0;
static long int motor_b_speed = 0;
// Value we used for the speed in the previous iteration.
static float last_speed = 0;
// true once the bot is standing up.
static boolean reset_complete = false;
// Counter, used to schedule work that we only do once in a while.
static long int loop_count = 0;
// true if an error has occurred like a timer overflow.
static boolean error = false;
// High water mark for timer values at the end of our ISR.
static long int max_timer = 0;
// True if the timer handler should periodically write out debugging information
// to the serial line.
static volatile bool write_gyro = false;
// Current gyroscope offset.  Set through calibration.
static float gyro_offset = GYRO_OFFSET;
// Current calibration factor applied to the accelerometer values.
static float accel_fact = 1.0;
// Loop counters.
static uint16_t counter = 0;
static int calibration_counter = 0;
// Ring buffer tracking last few g sq values.
#define NUM_G_SQ 3
static float total_gs_sq[NUM_G_SQ];
static int gs_idx = 0;

/**
 * (Timer 1 == OCR1A) interrupt handler.  Called each time timer 1 hits the top
 * value we set in OCR1A at start of day.
 *
 * The timer is automatically reset to 0 after it hits OCR1A.
 */
ISR(TIMER1_COMPA_vect)
{
  // Read the inputs.  Each analog read should take about 0.12 msec.  We can't
  // do too many analog reads per timer tick.
  // Gyro rate.
  gyro_reading = analogRead(gyro_pin);
  // Accelerometer
  x_reading = analogRead(x_pin);
  y_reading = analogRead(y_pin);

  // Convert readings to real units.
  gyro_rads_per_sec =  GYRO_RAD_PER_ADC_UNIT * (512 - gyro_reading) - gyro_offset;
  x_gs = ACCEL_G_PER_ADC_UNIT * (x_reading - 512);
  y_gs = ACCEL_G_PER_ADC_UNIT * (y_reading - 512);

  // Filter the inputs from the accelerometer to try to reduce the impact of
  // vibration and acceleration from any movements the bot is making.
  x_filt_gs = filterx(x_gs) * accel_fact;
  y_filt_gs = filtery(y_gs) * accel_fact;

  // Our constant rotation to apply to the accelerometer.  Sets the 0-tilt
  // angle.
  float accel_rotation = ((x_offset_pot - 512) * 0.001);

  // Estimate the tilt from the accelerometer alone.  We'll factor in a fraction
  // of this below.
  float accel_tilt_estimate = asin(max(min(x_filt_gs, 1.0), -1.0)) + accel_rotation;

  float gs_sq = (x_gs * x_gs + y_gs * y_gs);
  total_gs_sq[gs_idx++] = abs(1.0 - gs_sq * accel_fact * accel_fact);
  if (gs_idx == NUM_G_SQ)
  {
    gs_idx = 0;
  }

  static long int num_cal_readings;
  static float sum_gyro;
  static float sum_gs;

  if (digitalRead(switch_pin) == LOW && calibration_counter == 0)
  {
    // The switch was pressed, start the calibration counter
    Serial.print("Beginning calibration\n");
    calibration_counter = 10 * TARGET_LOOP_HZ;
    num_cal_readings = 0;
    sum_gyro = 0;
    sum_gs = 0;
  }

  if (calibration_counter == 1)
  {
    // Calibration complete, calculate the results.
    gyro_offset = sum_gyro / num_cal_readings;
    Serial.print("New gyro offset: ");
    Serial.print(gyro_offset, 4);
    Serial.print('\n');
    Serial.flush();
    write_float_to_eeprom(GYRO_OFFSET_ADDR, gyro_offset);

    accel_fact = 1.0 / (sum_gs / num_cal_readings);
    Serial.print("New accelerometer factor: ");
    Serial.print(accel_fact, 4);
    Serial.print('\n');
    Serial.flush();
    write_float_to_eeprom(ACCEL_FACT_ADDR, accel_fact);
  }
  else if (calibration_counter > 0 && calibration_counter < 8 * TARGET_LOOP_HZ)
  {
    // We're running a calibration and we've let some time pass since the
    // calibration started to let values settle.  Accumulate data.
    //
    // While calibrating, we should be still so finding the average gyro value
    // should give us its null point.
    sum_gyro += GYRO_RAD_PER_ADC_UNIT * (512 - gyro_reading);

    // Since we're still, the accelerometer should be measuring exactly 1G of
    // force.  Sum its magnitude so we can calculate the average.
    sum_gs += sqrt(gs_sq);
    num_cal_readings++;
  }
  else if (y_gs < 0.1 && abs(x_filt_gs) > 0.6)
  {
    // We fell over! Shut off the motors.
    speed = 0;
    reset_complete = false;
  }
  else if (!reset_complete)
  {
    // We've never been upright, wait until we're righted by the user.
    if (-0.02 < x_gs && x_gs < 0.02)
    {
      tilt_rads_estimate = accel_tilt_estimate;
      speed = 0;
      tilt_int_rads = 0;
      reset_complete = true;
    }
  }
  else
  {
    const float g_factor = 0.0005;
    tilt_rads_estimate = (1.0 - g_factor) * (tilt_rads_estimate + gyro_rads_per_sec * TICK_SECONDS) +
                         g_factor * accel_tilt_estimate;
    tilt_int_rads += tilt_rads_estimate * TICK_SECONDS;

#define D_TILT_FACT 200.0
#define TILT_FACT 5000.0
#define TILT_INT_FACT 50000.0

#define MAX_TILT_INT (0.015)

    if (tilt_int_rads > MAX_TILT_INT)
    {
      tilt_int_rads = MAX_TILT_INT;
    }
    if (tilt_int_rads < -MAX_TILT_INT)
    {
      tilt_int_rads = -MAX_TILT_INT;
    }

    speed = tilt_rads_estimate * TILT_FACT +
            tilt_int_rads * TILT_INT_FACT +
            gyro_rads_per_sec * D_TILT_FACT;

    if (TCNT1 > max_timer)
    {
      max_timer = TCNT1;
    }
  }

  // Set the motor directions
  digitalWrite(dir_a, speed < 0 ? LOW : HIGH);
  digitalWrite(dir_b, speed < 0 ? LOW : HIGH);

  if (write_gyro && counter >= 10)
  {
    Serial.print(accel_tilt_estimate, 4);
    Serial.print("\t");
    Serial.print(tilt_rads_estimate, 4);
    Serial.print("\t");
    Serial.print(tilt_int_rads, 4);
    Serial.print("\t");
    Serial.print(speed);
    Serial.print('\n');
    Serial.flush();
    counter = 0;
  }
  else
  {
    counter ++;
  }

  // Set the motor speeds.
  float abs_speed = 7 * sqrt(abs(speed));
  if (abs_speed > 0xff)
  {
    abs_speed = 0xff;
  }

  motor_a_speed = (long int)(abs_speed * MOTOR_A_FACTOR);
  motor_b_speed = (long int)(abs_speed * MOTOR_B_FACTOR);

  if (motor_a_speed > 0xff)
  {
    motor_a_speed = 0xff;
  }
  if (motor_b_speed > 0xff)
  {
    motor_b_speed = 0xff;
  }

  analogWrite(pwm_a, motor_a_speed);
  analogWrite(pwm_b, motor_b_speed);

  last_speed = speed;

  // Read the adjustment pots about once per second.  Stagger the reads to
  // avoid doing too many time-consuming reads in one loop.
  if (loop_count == TARGET_LOOP_HZ / 3)
  {
    d_tilt_pot = analogRead(A3);
  }
  else if (loop_count == (TARGET_LOOP_HZ * 2 /3))
  {
    x_offset_pot = analogRead(A5);
  }
  else if (loop_count == TARGET_LOOP_HZ)
  {
    gyro_offset_pot = analogRead(A4);
    loop_count = 0;
    if (error)
    {
      // There's been an error, turn on our status LED permanently.
      digitalWrite(led_pin, HIGH);
    }
    else
    {
      // No error, toggle the status LED once per second as a heartbeat.
      digitalWrite(led_pin, !digitalRead(led_pin));
    }
  }

  if (calibration_counter > 0)
  {
    calibration_counter--;
  }
  if (bit_is_set(TIFR1, OCF1A))
  {
    // This loop took too long and the timer wrapped.  That means that our
    // timing will be incorrect.
    error = true;
  }
  loop_count++;
}

/**
 * Start-of-day initialization.
 */
void setup()
{
  // Set control pins to be outputs
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  // Turn on the internal pullup for our switch input.
  pinMode(switch_pin, INPUT);
  PORTB |= _BV(PORTB0);

  // Turn off the motors for now.
  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);

  // Configure serial port for comms with BlueSMIRF.  It defaults to 115200
  // baud.
  Serial.begin(115200);

  // Read calibration data from EEPROM
  gyro_offset = read_float_from_eeprom(GYRO_OFFSET_ADDR);
  if (!(gyro_offset < 0.5 && gyro_offset > -0.5))
  {
    // Data didn't look sane, us a default.
    gyro_offset = GYRO_OFFSET;
  }
  accel_fact = read_float_from_eeprom(ACCEL_FACT_ADDR);
  if (!(accel_fact < 1.5 && accel_fact > 0.5))
  {
    // Data didn't look sane, us a default.
    accel_fact = 1.0;
  }

  // Timer0 PWM
  TCCR0B = (TCCR0B & 0xf8) | _BV(CS02);
  TCCR2B = (TCCR0B & 0xf8) | _BV(CS02);

  // We use Timer 1 as our accurate timebase.
  // Set it to CTC mode so that we can configure its TOP value in OCR1A.
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  // Enable (Timer 1 == OCR1A) interrupt.
  TIMSK1 = _BV(OCIE1A);
  // Have to set the TOP value *after* setting up the timer or it doesn't take
  // effect.
  OCR1A = TICKS_PER_LOOP;

  // Enable interrupts
  sei();
}

/**
 * The standard loop function, called repeatedly.  Most of the heavy lifting is
 * done by the interrupt routine above.  We only use the loop function to read
 * from serial.
 */
void loop()
{
  int c = Serial.read();
  if (c == 'g')
  {
    write_gyro = true;
  }
}
