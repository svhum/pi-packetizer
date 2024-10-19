#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include "si5351.h"

#define SI5351_I2C_ADDRESS         0x60
#define TXENPIN 23    // TXEN is BCM pin 23 = physical pin 16
#define KEYPIN 24     // KEY is BCM pin 24 = physical pin 18

Si5351 si5351(SI5351_I2C_ADDRESS);

int main(void) {
  // Initialize GPIO
  // Uses BCM numbering of the GPIOs and directly accesses the GPIO registers
  wiringPiSetupGpio();
  //pinMode(RESETPIN, OUTPUT);
  pinMode(TXENPIN, OUTPUT);
  pinMode(KEYPIN, OUTPUT);

  // Set to TX
  digitalWrite (TXENPIN, HIGH);
  digitalWrite (KEYPIN, HIGH);

	unsigned long long ullFreqHz = 0;
	si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  //si5351.set_pll_input(SI5351_PLLA, SI5351_PLL_INPUT_CLKIN);
  //si5351.set_pll_input(SI5351_PLLB, SI5351_PLL_INPUT_CLKIN);
  // Test of  single-ended output
	si5351.set_freq(1407400000ULL, SI5351_CLK2);		// format: (desired_freq_hz*100, OUTPUT_PIN)
	//si5351.set_freq(1400000000ULL, SI5351_CLK0);		// format: (desired_freq_hz*100, OUTPUT_PIN)
	//si5351.set_freq(1400000000ULL, SI5351_CLK1);		// format: (desired_freq_hz*100, OUTPUT_PIN)
	//si5351.set_freq(1407400000ULL, SI5351_CLK2);		// format: (desired_freq_hz*100, OUTPUT_PIN)
  //si5351.output_enable(SI5351_CLK2, 1);
	//si5351.set_freq(500000000ULL, SI5351_CLK2);		// format: (desired_freq_hz*100, OUTPUT_PIN)

	// Test of push-pull output
	// NOTE: frequency setting must take place FIRST before setting phases, inversion!
//si5351.set_freq(1407550000ULL, SI5351_CLK0);		// format: (desired_freq_hz*100, OUTPUT_PIN)
//	si5351.set_freq(1407550000ULL, SI5351_CLK1);		// format: (desired_freq_hz*100, OUTPUT_PIN)
//	si5351.set_phase(SI5351_CLK0, 0);
//	si5351.set_phase(SI5351_CLK1, 0);
//  si5351.pll_reset(SI5351_PLLA);
//  si5351.set_clock_invert(SI5351_CLK0, 0);
//  si5351.set_clock_invert(SI5351_CLK1, 1);

	// Test of quadrature output
	//si5351.set_freq_manual(1407400000ULL, 70370000000ULL, SI5351_CLK0);		// format: (desired_freq_hz*100, OUTPUT_PIN)
	//si5351.set_freq_manual(1407400000ULL, 70370000000ULL, SI5351_CLK1);		// format: (desired_freq_hz*100, OUTPUT_PIN)
	//si5351.set_phase(SI5351_CLK0, 0);
	//si5351.set_phase(SI5351_CLK1, 50);

	si5351.update_status();
	usleep(500000);

	printf("SYS_INIT: %d\n", si5351.dev_status.SYS_INIT);
	printf("LOL_A: %d\n", si5351.dev_status.LOL_A);
	printf("LOL_B: %d\n", si5351.dev_status.LOL_B);
	printf("LOS: %d\n", si5351.dev_status.LOS);
	printf("REVID: %d\n", si5351.dev_status.REVID);

	return(0);
}
