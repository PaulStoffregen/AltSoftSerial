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
// AltSoftSerial Receive Test
//
// Transmit data with Serial1 and try to receive
// it with AltSoftSerial.  You must connect a wire
// from Serial1 TX to AltSoftSerial RX.

#include <AltSoftSerial.h>

AltSoftSerial altser;
const int mybaud = 9600;

// Board            Serial1 TX   AltSoftSerial RX
// -----            ----------   ----------------
// Teensy 3.x            1              20
// Teensy 2.0            8 (D3)         10 (C7)
// Teensy++ 2.0          3 (D3)          4 (D4)
// Arduino Leonardo      1              13
// Arduino Mega         18              48

// Serial1 on AVR @ 16 MHz minimum baud is 245
// Serial1 on Teensy 3.2 @ 96 MHz minimum baud is 733

byte sentbyte;
unsigned long prevmillis;
byte testbyte=0xF0;

void setup() {
  delay(200);
  Serial.begin(9600);
  while (!Serial) ;  // wait for Arduino Serial Monitor
  Serial1.begin(mybaud); // connect a wire from TX1
  altser.begin(mybaud);   // to AltSoftSerial RX
  Serial.println("AltSoftSerial Receive Test");
  prevmillis = millis();
}

void loop() {
  // transmit a test byte on Serial 1
  if (millis() - prevmillis > 250) {
    sentbyte = testbyte++;
    Serial1.write(sentbyte);
    prevmillis = millis();
  }
  // attempt to receive it by AltSoftSerial
  if (altser.available() > 0) {
    byte b = altser.read();
    Serial.println(b);
    if (b != sentbyte) Serial.println("***** ERROR *****");
  }
}
