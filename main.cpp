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

inline void take_sample();

static union {
    manchester_data_t data;
    char raw_data[sizeof(manchester_data_t)];
} manchester;

/**
 * (Timer 1 == OCR1A) interrupt handler.  Called each time timer 1 hits the top
 * value we set in OCR1A at start of day.
 *
 * The timer is automatically reset to 0 after it hits OCR1A.
 */
ISR(TIMER1_COMPA_vect)
{
    take_sample();
}

// Number of timer ticks per half-bit of manchester encoding.  From spreadsheet
// at https://docs.google.com/spreadsheet/ccc?key=0AkaHr7xXc1PldDFmZnowZ29nTWVNV2U1ZVRqbVlfSWc
#define TICKS_PER_LOOP 26664

static volatile bool packet_available;

/**
 * Start-of-day initialization.
 */
void setup()
{
  Serial.begin(115200);

  // We use Timer 1 as our accurate timebase.
  // Set it to CTC mode so that we can configure its TOP value in OCR1A.
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10) /* Clock-select full speed */;
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
    if (packet_available)
    {
        // Copy the packet from the input buffer to a local one, then reenable
        // the timer interrupt.
        manchester_data_t packet;
        memcpy(&packet, &manchester, sizeof(manchester_data_t));
        packet_available = false;
        sei();

        // Write the data to the console.
        Serial.print(packet.node_id, HEX);
        Serial.print(' ');
        Serial.print(packet.seq_no, DEC);
        Serial.print(' ');
        Serial.print(packet.reading_type);
        Serial.print(' ');
        Serial.print(packet.reading, HEX);
        Serial.print('\n');
    }
}

typedef enum {
    rx_state_sync,
    rx_state_first_half_bit,
    rx_state_second_half_bit
} rx_state_t;

static rx_state_t rx_state = rx_state_sync;

#define RX_PIN 7
#define SYNC_PATTERN 0x55AA

inline void take_sample()
{
    static uint16_t recent_samples;
    static char first_half_bit;
    static uint8_t write_idx;
    static uint8_t write_bit_idx;
    char sample = digitalRead(RX_PIN);
    Serial.print(sample ? '1' : '0');

    recent_samples <<= 1;
    recent_samples |= sample ? 1 : 0;

    if (rx_state == rx_state_sync)
    {
        // Waiting for the next packet, which starts with an alternating
        // sync pattern. Keep track of the last 16 samples and check for the
        // sync pattern.
        if (recent_samples == SYNC_PATTERN)
        {
            // Found the sync pattern, the next bit should be data.
            Serial.println("SYNC");
            recent_samples = 0;
            rx_state = rx_state_first_half_bit;
            write_bit_idx = 0;
            write_idx = 0;
        }
    }
    else if (rx_state == rx_state_first_half_bit)
    {
        // Read the first half of a Manchester-encoded bit.  We don't process
        // it until we get the second half of the bit.
        first_half_bit = sample;
        rx_state = rx_state_second_half_bit;
    }
    else if (rx_state == rx_state_second_half_bit)
    {
        // Read the second half of the bit.
        rx_state = rx_state_first_half_bit;
        if (first_half_bit && !sample)
        {
            // High then low, a Manchester-encoded 1-bit.
            manchester.raw_data[write_idx] |= (char)(1 << write_bit_idx);
        }
        else if (!first_half_bit && sample)
        {
            // Low then high, a Manchester-encoded 0-bit.
            manchester.raw_data[write_idx] &= (char)~(1 << write_bit_idx);
        }
        else
        {
            // Error, the first and second half of the bits should always be
            // different.  Abandon the packet.
            Serial.println("ERROR");
            rx_state = rx_state_sync;
            return;
        }

        write_bit_idx += 1;
        if (write_bit_idx >= 8)
        {
            // We've received a whole byte, move to the next one.
            Serial.print("BYTE ");
            Serial.println((int)manchester.raw_data[write_idx]);
            write_bit_idx = 0;
            write_idx += 1;
            if (write_idx >= sizeof(manchester_data_t))
            {
                // We've received a whole packet.  Process the packet and
                // return to the sync state to wait for a new one.
                Serial.print("\n*PACK*\n");
                write_idx = 0;
                rx_state = rx_state_sync;
                packet_available = true;
                cli();
            }
        }
    }
}

