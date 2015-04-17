#include "LCAnalyzeFFT.h"

LCAnalyzeFFT LCFFT;

void setup() {
	Serial.begin(9600);
	delay(2000);
	Serial.println("hello");

	// Initialize the library with the appropriate input pin
	LCFFT.init(A2);
	// enable sampling
	LCFFT.enable();
} 

int counter = 0;
void loop() {
	static unsigned long start = millis();
	// wait until data is available
	if(LCFFT.available()) {
		counter++;
		if((millis() - start) > 1000) {
			Serial.println(counter);
			counter = 0;
			start = millis();
		}
		for(int i = 0 ;i < 32; i++) {
			int32_t sum = 0;
			for(int j = 0; j < 4; j++) {
				sum += LCFFT.output[i*4+j];
			}
			Serial.print(sum);
			Serial.print(" | ");
		}
		Serial.println();
	}
}
