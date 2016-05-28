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
static uint16_t cycles_per_tick=0;
static uint16_t ticks_to_exit=1; //if prescale > 8
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


#ifndef INPUT_PULLUP
#define INPUT_PULLUP INPUT
#endif

#define MAX_COUNTS_PER_BIT  6241  // 65536 / 10.5

uint16_t AltSoftSerial::ticksPerBit()
{
	return ticks_per_bit;
}
uint16_t AltSoftSerial::cyclesPerTick()
{
	return cycles_per_tick;
}

bool AltSoftSerial::prescale(uint32_t cycles_per_bit)
{
	timing_error = false;
	ticks_per_bit = cycles_per_bit;
	ticks_to_exit = 1;
	if (ticks_per_bit < MAX_COUNTS_PER_BIT) {
		//Serial.printf("cycles_per_bit = %d\n", cycles_per_bit);
		cycles_per_tick = 1;
		ticks_to_exit = 64;
		CONFIG_TIMER_NOPRESCALE();
		return true;
	}
	ticks_per_bit = cycles_per_bit / 8;
	if (ticks_per_bit < MAX_COUNTS_PER_BIT) {
		//Serial.printf("cycles_per_bit/8 = %d\n", cycles_per_bit);
		cycles_per_tick = 8;
		ticks_to_exit = 8;
		CONFIG_TIMER_PRESCALE_8();
		return true;
	}
#if defined(CONFIG_TIMER_PRESCALE_128)
	ticks_per_bit = cycles_per_bit / 128;
	if (ticks_per_bit < MAX_COUNTS_PER_BIT) {
		//Serial.printf("cycles_per_bit/64 = %d\n", cycles_per_bit);
		cycles_per_tick = 128;
		CONFIG_TIMER_PRESCALE_128();
		return true;
	}
#endif
#if defined(CONFIG_TIMER_PRESCALE_256)
	ticks_per_bit = cycles_per_bit / 256;
	if (ticks_per_bit < MAX_COUNTS_PER_BIT) {
		//Serial.printf("cycles_per_bit/256 = %d\n", cycles_per_bit);
		cycles_per_tick = 256;
		CONFIG_TIMER_PRESCALE_256();
		return true;
	}
#endif
#if defined(CONFIG_TIMER_PRESCALE_1024)
	ticks_per_bit = cycles_per_bit / 1024;
	if (ticks_per_bit < MAX_COUNTS_PER_BIT) {
		//Serial.printf("cycles_per_bit/1024 = %d\n", cycles_per_bit);
		cycles_per_tick = 1024;
		CONFIG_TIMER_PRESCALE_1024();
		return true;
	}
#endif
	timing_error = true;
	//Serial.println("baudrate too low");
	return false; //baudrate too low
}

bool AltSoftSerial::init(uint32_t cycles_per_bit)
{
	if (!prescale(cycles_per_bit)) return false;
	rx_stop_ticks = cycles_per_bit * 37 / 4;
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
	return true;
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
	while (state < 10) {
		target += ticks_per_bit;
		if (state < 9)
			bit = byte & 1;
		else
			bit = 1; // stopbit
		byte >>= 1;
		state++;
		if (bit != tx_bit) {
			if (bit) {
				CONFIG_MATCH_SET();
			} else {
				CONFIG_MATCH_CLEAR();
			}
			tx_bit = bit;
			tx_byte = byte;
			tx_state = state;
			SET_COMPARE_A(target);
			// TODO: how to detect timing_error?
			return;
		}
	}
	head = tx_buffer_head;
	tail = tx_buffer_tail;
	if (head == tail) {
		if (state == 10) {
			// Wait for final stop bit to finish
			tx_state = 11;
			SET_COMPARE_A(target + ticks_per_bit + ticks_to_exit);
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
		tx_state = 1;
		CONFIG_MATCH_CLEAR();
		if (state == 10)
			SET_COMPARE_A(target + ticks_per_bit + ticks_to_exit);
		else
			SET_COMPARE_A(GET_TIMER_COUNT() + ticks_to_exit);
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
			rx_byte = (rx_byte >> 1) | rx_bit;
			target += ticks_per_bit;
			state++;
			if (state >= 9) {
				DISABLE_INT_COMPARE_B();
				head = rx_buffer_head + 1;
				if (head >= RX_BUFFER_SIZE) head = 0;
				if (head != rx_buffer_tail) {
					rx_buffer[head] = rx_byte;
					rx_buffer_head = head;
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
	while (state < 9) {
		rx_byte = (rx_byte >> 1) | bit;
		state++;
	}
	head = rx_buffer_head + 1;
	if (head >= RX_BUFFER_SIZE) head = 0;
	if (head != rx_buffer_tail) {
		rx_buffer[head] = rx_byte;
		rx_buffer_head = head;
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

void AltSoftSerial::flushInput(void)
{
	rx_buffer_head = rx_buffer_tail;
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
