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

#include "LC_ADC.h"
#include <IntervalTimer.h>


#define ADC_VERY_LOW_SPEED      0
#define ADC_LOW_SPEED           1
#define ADC_MED_SPEED           2
#define ADC_HIGH_SPEED_16BITS   3
#define ADC_HIGH_SPEED          4
#define ADC_VERY_HIGH_SPEED     5

static bool bufferFull;
static uint16complex_t * externalBuffer;
static int externalBufferLength;
IntervalTimer timer0; // timers
static int pin;
volatile int bufferPosition = 0;

// is set to 1 when the calibration procedure is taking place
static uint8_t calibrating;

// the first calibration will use 32 averages and lowest speed,
// when this calibration is over the averages and speed will be set to default.
static uint8_t init_calib;
static uint16_t dc_average;

static void timer0_callback();
static void enableInterrupts();
static void wait_for_cal();
static void calibrateADCOffset();
static void setConversionSpeed(uint8_t);
static void setSamplingSpeed(uint8_t);
static void (*bufferFullCb)() = NULL;
static const uint8_t channel2sc1a[] = { // new version, gives directly the sc1a number. 0x1F=31 deactivates the ADC.
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, 0, 19, 3, 21, // 0-13, we treat them as A0-A13
	5, 14, 8, 9, 13, 12, 6, 7, 15, 4, // 14-23 (A0-A9)
	31, 31, 31, 31, 31, 31, 31, 31, 31, 31, // 24-33
	0, 19, 3, 21, // 34-37 (A10-A13)
	26, 22, 23, 27, 29, 30 // 38-43: temp. sensor, VREF_OUT, A14, bandgap, VREFH, VREFL. A14 isn't connected to anything in Teensy 3.0.
};
void initADC(int adcPin, void (*bufferFullCallback)(), uint16complex_t * buffer, int len) {
	calibrating = false;
	init_calib = true;
	bufferFull = false;
	bufferFullCb = bufferFullCallback;
	externalBuffer = buffer;
	externalBufferLength = len;
	bufferPosition = 0;
	pin = adcPin;
	pinMode(adcPin, INPUT);
	// perform a single internal read to get things setup
	analogRead(adcPin);
	///// ADC0 ////
	analogReference(INTERNAL);

	analogReadAveraging(8); // set number of averages
	analogReadResolution(12); // set bits of resolution

	// it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
	// see the documentation for more information
	setConversionSpeed(ADC_HIGH_SPEED);
	setSamplingSpeed(ADC_HIGH_SPEED);
	delay(100);
	calibrateADCOffset();
	enableInterrupts();
}

void calibrateADCOffset() {
	uint32_t sum = 0;
	for (int i=0; i < 32767; i++) {
		sum += analogRead(pin);
	}

	setDCOffset(sum >> 15);

}
void setDCOffset(uint16_t newDcOffset) { 
	dc_average = newDcOffset;
}

uint16_t getDCOffset() {
	return dc_average;
}

bool startReading(int period) {
	bufferFull = false;
	bufferPosition = 0;
	// start the timers, if it's not possible, startTimerValuex will be false
	return timer0.begin(timer0_callback, period);
}

void stopReading() {
	timer0.end();
}

static void startSingleRead() {
	ADC0_SC3 &= ~ADC_SC3_ADCO;
	// select pin for single-ended mode and start conversion, enable interrupts to know when it's done
	__disable_irq();
	ADC0_SC1A = channel2sc1a[pin] | ADC_SC1_AIEN;
	__enable_irq();
}

void timer0_callback() {
	startSingleRead();
}

void enableInterrupts() {
    ADC0_SC1A |= ADC_SC1_AIEN;
    NVIC_ENABLE_IRQ(IRQ_ADC0);
}

static inline int readSingle() {
	return (int16_t)ADC0_RA - dc_average;
}

bool isBufferFull() {
	return bufferFull;
}

//// when the measurement finishes, this will be called
//// first: see which pin finished and then save the measurement into the correct buffer
void adc0_isr() {
#ifdef SCOPE_FFT
	digitalWriteFast(13, HIGH);
#endif
	if(bufferPosition == externalBufferLength) {
		bufferPosition = 0;
		bufferFull = true;
		stopReading();
		if(bufferFullCb != NULL)
			bufferFullCb();
	}
	// read into the real component
	externalBuffer[bufferPosition].real = readSingle();
	externalBuffer[bufferPosition].imaginary = 0;
	bufferPosition++;
#ifdef SCOPE_FFT
	digitalWriteFast(13, LOW);
#endif
}

void wait_for_cal(void)
{
	uint16_t sum;

	while(ADC0_SC3 & ADC_SC3_CAL) { // Bit ADC_SC3_CAL in register ADC0_SC3 cleared when calib. finishes.

	}

	__disable_irq();
	if (calibrating) {
		sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
		sum = (sum / 2) | 0x8000;
		ADC0_PG = sum;

		sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
		sum = (sum / 2) | 0x8000;
		ADC0_MG = sum;

		calibrating = 0;
	}
	__enable_irq();

	// the first calibration uses 32 averages and lowest speed,
	// when this calibration is over, set the averages and speed to default.
	if(init_calib) {

		// set conversion speed to medium
		setConversionSpeed(ADC_MED_SPEED);

		// set sampling speed to medium
		setSamplingSpeed(ADC_MED_SPEED);

		// number of averages to 4
		analogReadAveraging(4);

		init_calib = 0; // clear
	}

}

void calibrate() {

	__disable_irq();

	calibrating = 1;
	ADC0_SC3 &= ~ADC_SC3_CAL; // stop possible previous calibration
	ADC0_SC3 |= ADC_SC3_CALF; // clear possible previous error

	ADC0_SC3 |= ADC_SC3_CAL; // start calibration

	__enable_irq();
}



#if F_BUS == 48000000
  #define ADC_CFG1_3MHZ   (ADC_CFG1_ADIV(3) | ADC_CFG1_ADICLK(1)) // Clock divide select: 3=div8 + Input clock: 1=bus/2
  #define ADC_CFG1_6MHZ   (ADC_CFG1_ADIV(2) | ADC_CFG1_ADICLK(1)) // Clock divide select: 2=div4 + Input clock: 1=bus/2
  #define ADC_CFG1_12MHZ  (ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(1)) // Clock divide select: 1=div2 Input clock: 1=bus/2
  #define ADC_CFG1_24MHZ  (ADC_CFG1_ADIV(0) | ADC_CFG1_ADICLK(1)) // this is way too fast, so accurancy is not guaranteed

  #define ADC_CFG1_VERY_LOW_SPEED ADC_CFG1_LOW_SPEED
  #define ADC_CFG1_LOW_SPEED (ADC_CFG1_3MHZ)
  #define ADC_CFG1_MED_SPEED (ADC_CFG1_6MHZ)
  #define ADC_CFG1_HI_SPEED_16_BITS (ADC_CFG1_12MHZ)
  #define ADC_CFG1_HI_SPEED (ADC_CFG1_12MHZ)
  #define ADC_CFG1_VERY_HIGH_SPEED (ADC_CFG1_24MHZ)
#elif F_BUS == 24000000
  #define ADC_CFG1_1_5MHZ   (ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(1))
  #define ADC_CFG1_3MHZ     (ADC_CFG1_ADIV(3) + ADC_CFG1_ADICLK(0)) // Clock divide select: 3=div8 + Input clock: 0=bus
  #define ADC_CFG1_6MHZ     (ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(0)) // Clock divide select: 2=div4 + Input clock: 0=bus
  #define ADC_CFG1_12MHZ    (ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0)) // Clock divide select: 1=div2 + Input clock: 0=bus
  #define ADC_CFG1_24MHZ    (ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0)) // this is way too fast, so accurancy is not guaranteed

  #define ADC_CFG1_VERY_LOW_SPEED (ADC_CFG1_1_5MHZ)
  #define ADC_CFG1_LOW_SPEED (ADC_CFG1_3MHZ)
  #define ADC_CFG1_MED_SPEED (ADC_CFG1_6MHZ)
  #define ADC_CFG1_HI_SPEED_16_BITS (ADC_CFG1_12MHZ)
  #define ADC_CFG1_HI_SPEED (ADC_CFG1_12MHZ)
  #define ADC_CFG1_VERY_HIGH_SPEED (ADC_CFG1_24MHZ)
#endif
#define CLOCK_MASK (ADC_CFG1_ADIV(3) | ADC_CFG1_ADICLK(3))

void setConversionSpeed(uint8_t speed) {
    if (calibrating) wait_for_cal();

	ADC0_CFG2 &= ~ADC_CFG2_ADACKEN; // disable the internal asynchronous clock

    uint32_t ADC_CFG1_speed; // store the clock and divisor

    if(speed == ADC_VERY_LOW_SPEED) {
		ADC0_CFG2 &= !ADC_CFG2_ADHSC; // high-speed config: add 2 ADCK
		ADC0_CFG1 |= ADC_CFG1_ADLPC; // no low power conf.

        ADC_CFG1_speed = ADC_CFG1_VERY_LOW_SPEED;

    } else if(speed == ADC_LOW_SPEED) {
		ADC0_CFG2 &= ~ADC_CFG2_ADHSC; // high-speed config: add 2 ADCK
		ADC0_CFG1 |= ADC_CFG1_ADLPC; // no low power conf.

        ADC_CFG1_speed = ADC_CFG1_LOW_SPEED;

    } else if(speed == ADC_MED_SPEED) {
		ADC0_CFG2 &= ~ADC_CFG2_ADHSC; // high-speed config: add 2 ADCK
		ADC0_CFG1 &= ~ADC_CFG1_ADLPC; // no low power conf.

        ADC_CFG1_speed = ADC_CFG1_MED_SPEED;

    } else if(speed == ADC_HIGH_SPEED_16BITS) {
		ADC0_CFG2 |= ADC_CFG2_ADHSC; // high-speed config: add 2 ADCK
		ADC0_CFG1 &= ~ADC_CFG1_ADLPC; // no low power conf.

        ADC_CFG1_speed = ADC_CFG1_HI_SPEED_16_BITS;

    } else if(speed == ADC_HIGH_SPEED) {
		ADC0_CFG2 |= ADC_CFG2_ADHSC; // high-speed config: add 2 ADCK
		ADC0_CFG1 &= ~ADC_CFG1_ADLPC; // no low power conf.

        ADC_CFG1_speed = ADC_CFG1_HI_SPEED;

    } else if(speed == ADC_VERY_HIGH_SPEED) { // this speed is most likely out of specs, so accurancy can be bad
		ADC0_CFG2 |= ADC_CFG2_ADHSC; // high-speed config: add 2 ADCK
		ADC0_CFG1 &= ~ADC_CFG1_ADLPC; // no low power conf.

        ADC_CFG1_speed = ADC_CFG1_VERY_HIGH_SPEED;

    } else {
        return;
    }

	ADC0_CFG1 = (ADC0_CFG1 & ~CLOCK_MASK) | ADC_CFG1_speed;

}


// Sets the sampling speed
/** Increase the sampling speed for low impedance sources, decrease it for higher impedance ones.
* \param speed can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    ADC_VERY_LOW_SPEED is the lowest possible sampling speed (+24 ADCK).
    ADC_LOW_SPEED adds +16 ADCK.
    ADC_MED_SPEED adds +10 ADCK.
    ADC_HIGH_SPEED (or ADC_HIGH_SPEED_16BITS) adds +6 ADCK.
    ADC_VERY_HIGH_SPEED is the highest possible sampling speed (0 ADCK added).
* It doesn't recalibrate at the end.
*/
#define ADLSTSMASK (ADC_CFG2_ADLSTS(3))
void setSamplingSpeed(uint8_t speed) {
    if (calibrating) wait_for_cal();

    // Select between the settings
    if(speed == ADC_VERY_LOW_SPEED) {
		ADC0_CFG1 |= ADC_CFG1_ADLSMP;
		ADC0_CFG2 =  (ADC0_CFG2 & ~ADLSTSMASK) | ADC_CFG2_ADLSTS(0);

    } else if(speed == ADC_LOW_SPEED) {
		ADC0_CFG1 |= ADC_CFG1_ADLSMP;
		ADC0_CFG2 =  (ADC0_CFG2 & ~ADLSTSMASK) | ADC_CFG2_ADLSTS(1);

    } else if(speed == ADC_MED_SPEED) {
		ADC0_CFG1 |= ADC_CFG1_ADLSMP;
		ADC0_CFG2 =  (ADC0_CFG2 & ~ADLSTSMASK) | ADC_CFG2_ADLSTS(2);

    } else if( (speed == ADC_HIGH_SPEED) || (speed == ADC_HIGH_SPEED_16BITS) ) {
		ADC0_CFG1 |= ADC_CFG1_ADLSMP;
		ADC0_CFG2 =  (ADC0_CFG2 & ~ADLSTSMASK) | ADC_CFG2_ADLSTS(3);

    } else if(speed == ADC_VERY_HIGH_SPEED) {
		ADC0_CFG1 &= ~ADC_CFG1_ADLSMP;

    }

}