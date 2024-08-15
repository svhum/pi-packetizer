#include <stdio.h>
#include "si5351.h"

#define SI5351_I2C_ADDRESS         0x60

Si5351 si5351(SI5351_I2C_ADDRESS);

int main(void) {
	unsigned long long ullFreqHz = 0;
	si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
	si5351.set_freq(1200000000ULL, SI5351_CLK0);		// format: (desired_freq_hz*100, OUTPUT_PIN)

	return(0);
}
