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

#include <AltSoftSerial.h>

// AltSoftSerial show configuration example
// Will print library configuration to default serial port
// Open your serial monitor to see config for your board
// Printout would repeat every 10sec (just in case you missed it somehow)

// Print Configuration
// -----------------------

// Direct include AltSoftSerial internals to obtaion PIN setup (you do not need this in regular code)
#include <config/AltSoftSerial_Boards.h>

void printAltSoftSerialSetup(Stream &port)
{
  #define PRINT_PFX "AltSoftSerial:"
  #define PRINT_PIN_NAME(pin,name) { char buffer[128+1]; sprintf(buffer, PRINT_PFX "PIN:%2d %s", (int)pin, (const char*)name); port.println(buffer); }

  port.println(PRINT_PFX "Setup info: begin");

  #if defined(ALTSS_USE_FTM0)
  port.println(PRINT_PFX "USE FTM0");
  #endif

  #if defined(ALTSS_USE_TIMER1)
  port.println(PRINT_PFX "USE TIMER1");
  #endif

  #if defined(ALTSS_USE_TIMER2)
  port.println(PRINT_PFX "USE TIMER2");
  #endif

  #if defined(ALTSS_USE_TIMER3)
  port.println(PRINT_PFX "USE TIMER3");
  #endif

  #if defined(ALTSS_USE_TIMER4)
  port.println(PRINT_PFX "USE TIMER4");
  #endif

  #if defined(ALTSS_USE_TIMER5)
  port.println(PRINT_PFX "USE TIMER5");
  #endif

  #if defined(INPUT_CAPTURE_PIN)
  PRINT_PIN_NAME(INPUT_CAPTURE_PIN,"RX");
  #endif

  #if defined(OUTPUT_COMPARE_A_PIN)
  PRINT_PIN_NAME(OUTPUT_COMPARE_A_PIN,"TX");
  #endif

  #if defined(OUTPUT_COMPARE_B_PIN)
  PRINT_PIN_NAME(OUTPUT_COMPARE_B_PIN,"(unused PWM)");
  #endif

  #if defined(OUTPUT_COMPARE_C_PIN)
  PRINT_PIN_NAME(OUTPUT_COMPARE_C_PIN,"(unused PWM)");
  #endif

  #if defined(OUTPUT_COMPARE_D_PIN)
  PRINT_PIN_NAME(OUTPUT_COMPARE_D_PIN,"(unused PWM)");
  #endif

  #if defined(OUTPUT_COMPARE_E_PIN)
  PRINT_PIN_NAME(OUTPUT_COMPARE_E_PIN,"(unused PWM)");
  #endif

  #if defined(OUTPUT_COMPARE_F_PIN)
  PRINT_PIN_NAME(OUTPUT_COMPARE_F_PIN,"(unused PWM)");
  #endif

  port.println(PRINT_PFX "Setup info: end");

  #undef  PRINT_PIN_NAME
  #undef  PRINT_PFX
}

void setup()
{
  // Open default serial to dump config to
  Serial.begin(9600);
  while (!Serial) ; // wait for serial monitor
  printAltSoftSerialSetup(Serial);
}

void loop()
{
  // Repeat every 10 sec (just in case)
  delay(10000);
  Serial.println("");
  printAltSoftSerialSetup(Serial);
}

