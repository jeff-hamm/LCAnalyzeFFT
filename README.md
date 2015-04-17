# LCAnalyzeFFT
Provides a 256 sample, 22050hz FFT for the Teensy-LC platform.

This library is heavily based off Paul Stoffregen's audio 
library: (https://github.com/PaulStoffregen/Audio) and Pedro 
Villanueva's excellent ADC library for Teensy3 and Teensy3.1 
(https://github.com/pedvide/ADC).

## Installation
Just clone/unzip to your arduino/libraries folder. Check out the 
example for usage. 

## Usage
This library exposes an extremely simple interface for a 256
sample FFT on the Teensy-LC platform. Simply initialize to
a pin _LCAnalyzeFFT.init(pin)_. enable reads 
_LCAnalyzeFFT.enable()_. And wait for data to show in 
_LCAnalyzeFFT.output_, by checking 
_LCAnalyzeFFT.available()_.

## Notes
The Teensy-LC has sigificantly lower memory and 
clock rate than the Teensy 3/3.1. Furthermore, it is missing
a great number of features that improve the performance of
DSP code like this. The library currently uses a significant
percentage of the memory and processing time available to 
the device. That said, there should be enough processing 
power left over to do some cool stuff.

Of special note, by default, the implementation does not
use overlapping FFT windows, nor does it average transforms
together to create a more stable signal. If you would like to
use this feature, remove the _#define NOAVG_ symbol from
the LCAnalyzeFFT header. Note that there will be very little
memory remaining on the device for other features. 
