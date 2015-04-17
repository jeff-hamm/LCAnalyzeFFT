/* Teensy-LC FFT Analyzer Library
 * Copyright (c) 2015 Jeff Hamm, jeff.hamm@gmail.com
 *
 * Significant portions of the implementation of this file is based off
 * Pedro Villanueva's excellent ADC library for Teensy3 and Teensy3.1.
 * (https://github.com/pedvide/ADC)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INC_LC_ADC_H
#define INC_LC_ADC_H

#include <Arduino.h>
#include "LCAnalyzeFFT.h"

// public stuff
void initADC(int adcPin, void (*bufferFullCallback)(), uint16complex_t * buffer, int len); 
bool isBufferFull();
bool startReading(int period);
void stopReading();
volatile extern int bufferPosition;

#endif