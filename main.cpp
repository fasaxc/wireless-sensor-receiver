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

#include <avr/common.h>
#include <avr/io.h>
#include "WProgram.h"
#include <stdlib.h>
#include <stdarg.h>
#include "cppboilerplate.h"
#include <avr/interrupt.h>
#include <sensor_node.h>

inline void take_sample(uint16_t time, bool rising_edge);

/**
 * (Timer 1 == OCR1A) interrupt handler.  Called each time timer 1 hits the top
 * value we set in OCR1A at start of day.
 *
 * The timer is automatically reset to 0 after it hits OCR1A.
 */
ISR(TIMER1_CAPT_vect)
{
    /* Reset the counter. */
    TCNT1 = 0;
    /* Read the CC value ASAP. */
    uint16_t time = ICR1;
    /* Then, switch to looking for the opposite edge. */
    bool rising_edge = TCCR1B & _BV(ICES1);
    TCCR1B ^= _BV(ICES1);
    take_sample(time, rising_edge);
}

// Number of timer ticks per half-bit of manchester encoding.  From spreadsheet
// at https://docs.google.com/spreadsheet/ccc?key=0AkaHr7xXc1PldDFmZnowZ29nTWVNV2U1ZVRqbVlfSWc
#define TICKS_PER_LOOP (8000)

static volatile bool packet_available;

/**
 * Start-of-day initialization.
 */
void setup()
{
  Serial.begin(115200);

  pinMode(8, INPUT);

  // We use Timer 1 as input capture compare.
  TCCR1A = 0;
  TCCR1B = _BV(ICNC1) /* Input capture noise canceler enabled */ |
           _BV(CS10)  /* Timer enabled, full clock speed. */;
  // Enable timer1 capture interrupt.
  TIMSK1 = _BV(ICIE1);

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
    if (packet_available)
    {
        // Copy the packet from the input buffer to a local one, then reenable
        // the timer interrupt.
        manchester_data_t packet;
        memcpy(&packet, &manchester_union, sizeof(manchester_data_t));
        packet_available = false;
        sei();

        uint8_t checksum = calculate_checksum(&packet);

        // Write the data to the console.
        Serial.print((uint8_t)packet.node_id, HEX);
        Serial.print(' ');
        Serial.print((uint8_t)packet.seq_no, DEC);
        Serial.print(' ');
        Serial.print((uint8_t)packet.reading_type, HEX);
        Serial.print(' ');

        float t_mv = packet.reading * 1100.0 / 1024.0;
        float t = 25 + ((t_mv - 750.0) / 10.0);

        Serial.print(t, 2);
        Serial.print(' ');
        Serial.print(packet.checksum, HEX);
        Serial.print(' ');
        Serial.println(checksum, HEX);
    }
}

typedef enum {
    rx_state_wait,
    rx_state_sync,
    rx_state_receive,
    rx_state_receive_short
} rx_state_t;

static rx_state_t rx_state = rx_state_wait;

#define RX_PIN 8

#define TIME_IS_1T(time) (time >  TICKS_PER_LOOP * 0.5 && \
                          time < TICKS_PER_LOOP * 1.5)

#define TIME_IS_2T(time) (time >  TICKS_PER_LOOP * 1.5 && \
                          time < TICKS_PER_LOOP * 2.5)

#define MIN_PREAMBLE_LENGTH 16

static uint8_t write_idx;
static uint8_t write_bit_idx;

inline void on_receive_reset() {
    write_idx = 0;
    write_bit_idx = 0;
}

inline void on_bit_received(bool bit) {
    const uint8_t mask = (uint8_t)(1 << write_bit_idx);

    manchester_union.manchester_data[write_idx] &= ~mask;
    if (bit) {
        manchester_union.manchester_data[write_idx] |= mask;
    }

    write_bit_idx += 1;
    if (write_bit_idx >= 8)
    {
        // We've received a whole byte, move to the next one.
        write_bit_idx = 0;
        write_idx += 1;
        if (write_idx >= sizeof(manchester_data_t))
        {
            // We've received a whole packet.  Process the packet and
            // return to the sync state to wait for a new one.
            write_idx = 0;
            rx_state = rx_state_wait;
            packet_available = true;
        }
    }
}

inline void take_sample(uint16_t time, bool rising_edge)
{
    static uint8_t sync_bit_count;
    static bool last_bit;

    if (rx_state == rx_state_wait) {
        // Waiting for the first change of bit.  The timer value will be
        // garbage.
        if (!rising_edge) {
            // Got a falling edge, let's begin.
            rx_state = rx_state_sync;
            sync_bit_count = 0;
            on_receive_reset();
        }
    } else if (rx_state == rx_state_sync) {
        // Looking for the sync pattern, which consists of many 1T transitions
        // followed by one 2T transition.
        if (TIME_IS_1T(time)) {
            // Short pulse, just keep counting.
            sync_bit_count++;
            //Serial.print('.');
        } else if (TIME_IS_2T(time)) {
            if (sync_bit_count > MIN_PREAMBLE_LENGTH) {
                // We've seen plenty of 1T transitions and now a 2T one.  Looks
                // like we've got a packet.
                //Serial.println("SYNC");
                rx_state = rx_state_receive;
                last_bit = 0;
            }
        } else {
            // Error, transition time not in acceptable range.
            rx_state = rx_state_wait;
            //Serial.print(time < TICKS_PER_LOOP ? '<' : '>');
        }
    } else if (rx_state == rx_state_receive) {
        if (TIME_IS_1T(time)) {
            // Short pulse, this bit must be the same as the last bit.  We
            // expect another short pulse after this.
            rx_state = rx_state_receive_short;
            //Serial.print('s');
        } else if (TIME_IS_2T(time)) {
            // Long pulse, this bit must be opposite to previous bit.
            last_bit = !last_bit;
            on_bit_received(last_bit);
            //Serial.print('l');
        } else {
            // Error, transition time not in acceptable range.
            //Serial.print('E');
            rx_state = rx_state_wait;
        }
    } else if (rx_state == rx_state_receive_short) {
        // We received a short pulse, absorb the following short pulse.
        if (TIME_IS_1T(time)) {
            // Got the expected short pulse.
            on_bit_received(last_bit);
            rx_state = rx_state_receive;
            //Serial.print('S');
        } else {
            // Error, transition time not in acceptable range.
            rx_state = rx_state_wait;
            //Serial.print('E');
        }
    } else {
        rx_state = rx_state_wait;
    }
}

