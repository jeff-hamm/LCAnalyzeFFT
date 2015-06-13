/* FFT Analyzer for Teensy-LC
 * Copyright (c) 2015, Jeff Hamm, jeff.hamm@gmail.com
 *
 * Significant portions of the structure and implementation of this library 
 * are based on Paul Stoffregen's audio library.
 * (https://github.com/PaulStoffregen/Audio)
 * Paul's audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "LCAnalyzeFFT.h"
#include "Arduino.h"
#include "LC_ADC.h"
#include "sqrt_integer.h"
#include "LC_ADC.h"

#define US_FOR_HZ(h) (1000000/h)

LCAnalyzeFFT * LCAnalyzeFFT::instance = NULL;

inline int32_t complexSquareSum(uint16complex_t a, uint16complex_t b) 
{
	return (int32_t)a.real * (int32_t)b.real + (int32_t)a.imaginary * (int32_t)b.imaginary;
}  

void apply_window_to_fft_buffer(uint16complex_t *buffer, const int16_t *win)
{
	int16_t *buf = (int16_t *)buffer;

	for (int i=0; i < ANALYZE_FFT_SIZE; i++) {
		int32_t val = *buf * *win++;
		*buf = val >> 15;
		buf += 2;
	}

}

LCAnalyzeFFT::LCAnalyzeFFT()  :  window(AudioWindowHanning256),	 outputflag(false)
#ifndef NOAVG
	,naverage(4),count(0)
#endif
{
	arm_cfft_radix4_init_q15(&fft_inst, ANALYZE_FFT_SIZE, 0, 1);
}


void bufferFull_callback() {

	LCAnalyzeFFT::instance->update();
}


void LCAnalyzeFFT::init(int pin) {
#ifdef SCOPE_FFT
	pinMode(13, OUTPUT); 
	pinMode(14, OUTPUT);
#endif

#ifndef NOAVG
	// fill the second half of the buffer in averaging mode
	initADC(pin, bufferFull_callback, &sampleBuffer[FFT_OUTPUT_SIZE], FFT_OUTPUT_SIZE);
	memset(sampleBuffer,0,FFT_OUTPUT_SIZE*sizeof(uint16complex_t));
#else
	initADC(pin, bufferFull_callback, sampleBuffer, ANALYZE_FFT_SIZE);
#endif
	instance = this;
}

void LCAnalyzeFFT::update() {
	if(isBufferFull()) {
#ifdef SCOPE_FFT
		digitalWriteFast(14, HIGH);
#endif
		fft();
#ifdef SCOPE_FFT
		digitalWriteFast(14, LOW);
#endif
		startReading(sampleDelay);
	}
}

bool LCAnalyzeFFT::enable(int sampleRate) {
	this->sampleDelay = US_FOR_HZ(sampleRate);
	return startReading(sampleDelay);
}

void LCAnalyzeFFT::disable() {
	stopReading();
}

bool LCAnalyzeFFT::available() {
	if (outputflag == true) {
		outputflag = false;
		return true;
	}
	return false;
}
float LCAnalyzeFFT::read(unsigned int binNumber) {
	if (binNumber > 127) return 0.0;
	return (float)(output[binNumber]) * (1.0 / 16384.0);
}
float LCAnalyzeFFT::read(unsigned int binFirst, unsigned int binLast) {
	if (binFirst > binLast) {
		unsigned int tmp = binLast;
		binLast = binFirst;
		binFirst = tmp;
	}
	if (binFirst > 127) return 0.0;
	if (binLast > 127) binLast = 127;
	uint32_t sum = 0;
	do {
		sum += output[binFirst++];
	} while (binFirst < binLast);
	return (float)sum * (1.0 / 16384.0);
}
void LCAnalyzeFFT::windowFunction(const int16_t *w) {
	window = w;
}

void LCAnalyzeFFT::fft()
{
	if (window) apply_window_to_fft_buffer(sampleBuffer, window);
	arm_cfft_radix4_q15(&fft_inst, (int16_t*)sampleBuffer);
#ifndef NOAVG
	// G. Heinzel's paper says we're supposed to average the magnitude
	// squared, then do the square root at the end.
	if (count == 0) {
		for (int i=0; i < FFT_OUTPUT_SIZE; i++) {
			uint16complex_t tmp = sampleBuffer[i];
			uint32_t magsq = complexSquareSum(tmp, tmp);
			sum[i] = magsq / naverage;
		}
	} else {
		for (int i=0; i < FFT_OUTPUT_SIZE; i++) {
			uint16complex_t tmp = sampleBuffer[i];
			uint32_t magsq = complexSquareSum(tmp, tmp);
			sum[i] += magsq / naverage;
		}
	}
	if (++count == naverage) {
		count = 0;
		for (int i=0; i < FFT_OUTPUT_SIZE; i++) {
			output[i] = sqrt_uint32_approx(sum[i]);

		}
		outputflag = true;
	}
	memcpy(sampleBuffer, &sampleBuffer[FFT_OUTPUT_SIZE], FFT_OUTPUT_SIZE * sizeof(uint16complex_t));
#else
	for (int i=0; i < FFT_OUTPUT_SIZE; i++) {
		uint16complex_t tmp = sampleBuffer[i];
		output[i] = sqrt_uint32_approx(complexSquareSum(tmp, tmp));
	}
	outputflag = true;
#endif
}