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

#ifndef INC_FFT_LC_H
#define INC_FFT_LC_H
#define ARM_MATH_CM0
#include "arm_math.h"


#define ANALYZE_FFT_SIZE 256
#define FFT_OUTPUT_SIZE (ANALYZE_FFT_SIZE/2)

// If you remove this define you will get a much higher quality FFT which uses overlapping windows
// However, the memory and performance cost is significantly higher
#define NOAVG

extern "C" {
	extern const int16_t AudioWindowHanning256[];
	extern const int16_t AudioWindowBartlett256[];
	extern const int16_t AudioWindowBlackman256[];
	extern const int16_t AudioWindowFlattop256[];
	extern const int16_t AudioWindowBlackmanHarris256[];
	extern const int16_t AudioWindowNuttall256[];
	extern const int16_t AudioWindowBlackmanNuttall256[];
	extern const int16_t AudioWindowWelch256[];
	extern const int16_t AudioWindowHamming256[];
	extern const int16_t AudioWindowCosine256[];
	extern const int16_t AudioWindowTukey256[];
}


// holds a complex number
union __attribute__ ((__packed__))  uint16complex_t 
{
	struct {
		int16_t real;
		int16_t imaginary;
	};
	uint32_t unified;
};

class LCAnalyzeFFT {

public:
	LCAnalyzeFFT();
	void init(int pin);
	bool enable(int sampleRate = 22050);
	void disable();
	int16_t output[FFT_OUTPUT_SIZE];
	void update();
	bool available();
	float read(unsigned int binNumber);
	float read(unsigned int binFirst, unsigned int binLast);
	void windowFunction(const int16_t *w);

#ifndef NOAVG
	void averageTogether(uint8_t n) {
		if (n == 0) n = 1;
		naverage = n;
	}
#endif

	static LCAnalyzeFFT * instance;
private: 
	void fft();
	const int16_t *window;
	bool outputflag;
	uint16complex_t sampleBuffer[ANALYZE_FFT_SIZE] __attribute__ ((aligned (4)));
	int sampleDelay;
	arm_cfft_radix4_instance_q15 fft_inst;
#ifndef NOAVG
	uint32_t sum[FFT_OUTPUT_SIZE];
	uint8_t naverage;
	uint8_t count;
#endif
};

#endif