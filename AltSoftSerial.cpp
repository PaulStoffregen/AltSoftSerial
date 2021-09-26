/* An Alternative Software Serial Library
 * http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
 * Copyright (c) 2014 PJRC.COM, LLC, Paul Stoffregen, paul@pjrc.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Revisions are now tracked on GitHub
// https://github.com/PaulStoffregen/AltSoftSerial
//
// Version 1.2: Support Teensy 3.x
//
// Version 1.1: Improve performance in receiver code
//
// Version 1.0: Initial Release


#include "AltSoftSerial.h"
#include "config/AltSoftSerial_Boards.h"
#include "config/AltSoftSerial_Timers.h"

/****************************************/
/**          Initialization            **/
/****************************************/

static uint16_t ticks_per_bit=0;
bool AltSoftSerial::timing_error=false;

static uint8_t rx_state;
static uint8_t rx_byte;
static uint8_t rx_bit = 0;
static uint16_t rx_target;
static uint16_t rx_stop_ticks=0;
static volatile uint8_t rx_buffer_head;
static volatile uint8_t rx_buffer_tail;
#define RX_BUFFER_SIZE 80
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

static volatile uint8_t tx_state=0;
static uint8_t tx_byte;
static uint8_t tx_bit;
static volatile uint8_t tx_buffer_head;
static volatile uint8_t tx_buffer_tail;
#define TX_BUFFER_SIZE 68
static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint8_t tx_parity;

static uint8_t data_bits, stop_bits;
static uint8_t parity; // 0 for none, 1 for odd, 2 for even
static uint8_t total_bits, almost_total_bits; // these are sums calculated during .begin() to speed up the loop in ISR(CAPTURE_INTERRUPT)
static uint8_t byte_alignment;

#ifndef INPUT_PULLUP
#define INPUT_PULLUP INPUT
#endif

#define MAX_COUNTS_PER_BIT  6241  // 65536 / 10.5

void AltSoftSerial::init(uint32_t cycles_per_bit, uint8_t config)
{
	//Serial.printf("cycles_per_bit = %d\n", cycles_per_bit);
	if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
		CONFIG_TIMER_NOPRESCALE();
	} else {
		cycles_per_bit /= 8;
		//Serial.printf("cycles_per_bit/8 = %d\n", cycles_per_bit);
		if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
			CONFIG_TIMER_PRESCALE_8();
		} else {
#if defined(CONFIG_TIMER_PRESCALE_256)
			cycles_per_bit /= 32;
			//Serial.printf("cycles_per_bit/256 = %d\n", cycles_per_bit);
			if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
				CONFIG_TIMER_PRESCALE_256();
			} else {
				return; // baud rate too low for AltSoftSerial
			}
#elif defined(CONFIG_TIMER_PRESCALE_128)
			cycles_per_bit /= 16;
			//Serial.printf("cycles_per_bit/128 = %d\n", cycles_per_bit);
			if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
				CONFIG_TIMER_PRESCALE_128();
			} else {
				return; // baud rate too low for AltSoftSerial
			}
#else
			return; // baud rate too low for AltSoftSerial
#endif
		}
	}
	ticks_per_bit = cycles_per_bit;
	/* [2019-08-11, stattin42]: Length of frame depends on total number of symbols/bits.
	   Need to adjust for number of data bits and parity bits. If not, start bit may be missed
	   for short formats if msb is a 1. In this case there is no edge between data/parity and
	   stop bit --> no capture interrupt --> relies on timer interrupt. Have not looked into
	   aspects of longer formats.                                                                 */
//	rx_stop_ticks = cycles_per_bit * 37 / 4;
	setBitCounts(config);   // total_bits are calculated here
	rx_stop_ticks = cycles_per_bit * ((4*(total_bits))+1)/4;
	pinMode(INPUT_CAPTURE_PIN, INPUT_PULLUP);
	digitalWrite(OUTPUT_COMPARE_A_PIN, HIGH);
	pinMode(OUTPUT_COMPARE_A_PIN, OUTPUT);
	rx_state = 0;
	rx_buffer_head = 0;
	rx_buffer_tail = 0;
	tx_state = 0;
	tx_buffer_head = 0;
	tx_buffer_tail = 0;
	ENABLE_INT_INPUT_CAPTURE();
}

void AltSoftSerial::end(void)
{
	DISABLE_INT_COMPARE_B();
	DISABLE_INT_INPUT_CAPTURE();
	flushInput();
	flushOutput();
	DISABLE_INT_COMPARE_A();
	// TODO: restore timer to original settings?
}


/****************************************/
/**           Transmission             **/
/****************************************/

void AltSoftSerial::writeByte(uint8_t b)
{
	uint8_t intr_state, head;

	head = tx_buffer_head + 1;
	if (head >= TX_BUFFER_SIZE) head = 0;
	while (tx_buffer_tail == head) ; // wait until space in buffer
	intr_state = SREG;
	cli();
	if (tx_state) {
		tx_buffer[head] = b;
		tx_buffer_head = head;
	} else {
		tx_state = 1;
		tx_byte = b;
		tx_bit = 0;
		if (parity)
			tx_parity = parity_even_bit(b) == (parity==2);
		ENABLE_INT_COMPARE_A();
		CONFIG_MATCH_CLEAR();
		SET_COMPARE_A(GET_TIMER_COUNT() + 16);
	}
	SREG = intr_state;
}

ISR(COMPARE_A_INTERRUPT)
{
	uint8_t state, byte, bit, head, tail;
	uint16_t target;

	state = tx_state;
	byte = tx_byte;
	target = GET_COMPARE_A();
	while (state < 11) {
		target += ticks_per_bit;  // Bit start time
		if (state < 9) {
			bit = byte & 1; // data bit
			byte >>= 1;
		} else {
			if (state == 9) {
				bit = tx_parity; // parity bit
			} else {
			        bit = 1; // stop bit
			}
		}
		state++;
		if (state == (data_bits+1))
		  state = 9 + !parity;
		if (bit != tx_bit) { // schedule flip of output for bit starting at time target
			if (bit) {
				CONFIG_MATCH_SET();
			} else {
				CONFIG_MATCH_CLEAR();
			}
			SET_COMPARE_A(target);
			tx_bit = bit;
			tx_byte = byte;
			tx_state = state;
			// TODO: how to detect timing_error?
			return;
		}
	}
	head = tx_buffer_head;
	tail = tx_buffer_tail;
	if (head == tail) {
		if (state == 11) {
			// Wait for final stop bit to finish
			tx_state = 12;
			SET_COMPARE_A(target + (stop_bits * ticks_per_bit));
		} else {
			tx_state = 0;
			CONFIG_MATCH_NORMAL();
			DISABLE_INT_COMPARE_A();
		}
	} else {
		if (++tail >= TX_BUFFER_SIZE) tail = 0;
		tx_buffer_tail = tail;
		tx_byte = tx_buffer[tail];
		tx_bit = 0;
		if (parity)
			tx_parity = parity_even_bit(tx_byte) == (parity==2);
		CONFIG_MATCH_CLEAR();
		if (state == 11)
			SET_COMPARE_A(target + (stop_bits * ticks_per_bit));
		else
			SET_COMPARE_A(GET_TIMER_COUNT() + 16);
		tx_state = 1;
		// TODO: how to detect timing_error?
	}
}

void AltSoftSerial::flushOutput(void)
{
	while (tx_state) /* wait */ ;
}


/****************************************/
/**            Reception               **/
/****************************************/

ISR(CAPTURE_INTERRUPT)
{
	uint8_t state, bit, head;
	uint16_t capture, target;
	uint16_t offset, offset_overflow;

	capture = GET_INPUT_CAPTURE();
	bit = rx_bit;
	if (bit) {
		CONFIG_CAPTURE_FALLING_EDGE();
		rx_bit = 0;
	} else {
		CONFIG_CAPTURE_RISING_EDGE();
		rx_bit = 0x80;
	}
	state = rx_state;
	if (state == 0) {
		if (!bit) {
			uint16_t end = capture + rx_stop_ticks;
			SET_COMPARE_B(end);
			ENABLE_INT_COMPARE_B();
			rx_target = capture + ticks_per_bit + ticks_per_bit/2;
			rx_state = 1;
		}
	} else {
		target = rx_target;
		offset_overflow = 65535 - ticks_per_bit;
		while (1) {
			offset = capture - target;
			if (offset > offset_overflow) break;
			if (state <= data_bits) // only store data bits
			        rx_byte = (rx_byte >> 1) | rx_bit;
			target += ticks_per_bit;
			state++;
			if (state > almost_total_bits) {
				DISABLE_INT_COMPARE_B();
				/* [2019-08-11, stattin42]: Bits in rx_byte are in correct
				   position only for 8-bit data format. For less than 8 bits,
				   rx_byte needs to be shifted for LS bit to get to bit0 location.
				   Added byte_alignment variable for speed:                        */
				rx_byte = rx_byte>>byte_alignment;
				if (!parity || (parity_even_bit(rx_byte) == (parity==2)) == (bool)rx_bit) {
					head = rx_buffer_head + 1;
					if (head >= RX_BUFFER_SIZE) head = 0;
					if (head != rx_buffer_tail) {
						rx_buffer[head] = rx_byte;
						rx_buffer_head = head;
					}
				}
				CONFIG_CAPTURE_FALLING_EDGE();
				rx_bit = 0;
				rx_state = 0;
				return;
			}
		}
		rx_target = target;
		rx_state = state;
	}
	//if (GET_TIMER_COUNT() - capture > ticks_per_bit) AltSoftSerial::timing_error = true;
}

ISR(COMPARE_B_INTERRUPT)
{
	uint8_t head, state, bit;

	DISABLE_INT_COMPARE_B();
	CONFIG_CAPTURE_FALLING_EDGE();
	state = rx_state;
	bit = rx_bit ^ 0x80;
	while (state <= data_bits) {
		rx_byte = (rx_byte >> 1) | bit;
		state++;
	}
	/* [2019-08-11, stattin42]: Bits in rx_byte are in correct position only for 8-bit
	   data format. For less than 8 bits, rx_byte needs to be shifted for LS bit to get to
	   bit0 location. Added byte_alignment variable for speed:                                  */
	rx_byte = rx_byte>>byte_alignment;
	if (!parity || (parity_even_bit(rx_byte) == (parity==2)) == (bool)bit) {
		head = rx_buffer_head + 1;
		if (head >= RX_BUFFER_SIZE) head = 0;
		if (head != rx_buffer_tail) {
			rx_buffer[head] = rx_byte;
			rx_buffer_head = head;
		}
	}
	rx_state = 0;
	CONFIG_CAPTURE_FALLING_EDGE();
	rx_bit = 0;
}



int AltSoftSerial::read(void)
{
	uint8_t head, tail, out;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if (head == tail) return -1;
	if (++tail >= RX_BUFFER_SIZE) tail = 0;
	out = rx_buffer[tail];
	rx_buffer_tail = tail;
	return out;
}

int AltSoftSerial::peek(void)
{
	uint8_t head, tail;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if (head == tail) return -1;
	if (++tail >= RX_BUFFER_SIZE) tail = 0;
	return rx_buffer[tail];
}

int AltSoftSerial::available(void)
{
	uint8_t head, tail;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if (head >= tail) return head - tail;
	return RX_BUFFER_SIZE + head - tail;
}

int AltSoftSerial::availableForWrite(void)
{ 
	uint8_t head, tail;
	head = tx_buffer_head;
	tail = tx_buffer_tail;

	if (tail > head) return tail - head;
	return TX_BUFFER_SIZE + tail - head;
}

void AltSoftSerial::flushInput(void)
{
	rx_buffer_head = rx_buffer_tail;
}

void AltSoftSerial::setBitCounts(uint8_t config) {
	parity = 0;
	stop_bits = 1;
	switch (config) {
	case SERIAL_5N1:
		data_bits = 5;
		break;
	case SERIAL_6N1:
		data_bits = 6;
		break;
	case SERIAL_7N1:
		data_bits = 7;
		break;
	case SERIAL_8N1:
		data_bits = 8;
		break;
	case SERIAL_5N2:
		data_bits = 5;
		stop_bits = 2;
		break;
	case SERIAL_6N2:
		data_bits = 6;
		stop_bits = 2;
		break;
	case SERIAL_7N2:
		data_bits = 7;
		stop_bits = 2;
		break;
	case SERIAL_8N2:
		data_bits = 8;
		stop_bits = 2;
		break;
	case SERIAL_5O1:
		parity = 1;
		data_bits = 5;
		break;
	case SERIAL_6O1:
		parity = 1;
		data_bits = 6;
		break;
	case SERIAL_7O1:
		parity = 1;
		data_bits = 7;
		break;
	case SERIAL_8O1:
		parity = 1;
		data_bits = 8;
		break;
	case SERIAL_5O2:
		parity = 1;
		data_bits = 5;
		stop_bits = 2;
		break;
	case SERIAL_6O2:
		parity = 1;
		data_bits = 6;
		stop_bits = 2;
		break;
	case SERIAL_7O2:
		parity = 1;
		data_bits = 7;
		stop_bits = 2;
		break;
	case SERIAL_8O2:
		parity = 1;
		data_bits = 8;
		stop_bits = 2;
		break;
	case SERIAL_5E1:
		parity = 2;
		data_bits = 5;
		break;
	case SERIAL_6E1:
		parity = 2;
		data_bits = 6;
		break;
	case SERIAL_7E1:
		parity = 2;
		data_bits = 7;
		break;
	case SERIAL_8E1:
		parity = 2;
		data_bits = 8;
		break;
	case SERIAL_5E2:
		parity = 2;
		data_bits = 5;
		stop_bits = 2;
		break;
	case SERIAL_6E2:
		parity = 2;
		data_bits = 6;
		stop_bits = 2;
		break;
	case SERIAL_7E2:
		parity = 2;
		data_bits = 7;
		stop_bits = 2;
		break;
	case SERIAL_8E2:
		parity = 2;
		data_bits = 8;
		stop_bits = 2;
		break;
	}

	total_bits = data_bits + (parity ? 1 : 0) + stop_bits;
	almost_total_bits = total_bits - stop_bits;
	byte_alignment = 8-data_bits;
}

#ifdef ALTSS_USE_FTM0
void ftm0_isr(void)
{
	uint32_t flags = FTM0_STATUS;
	FTM0_STATUS = 0;
	if (flags & (1<<0) && (FTM0_C0SC & 0x40)) altss_compare_b_interrupt();
	if (flags & (1<<5)) altss_capture_interrupt();
	if (flags & (1<<6) && (FTM0_C6SC & 0x40)) altss_compare_a_interrupt();
}
#endif

