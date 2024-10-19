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
  digitalWrite (TXENPIN, LOW);
  digitalWrite (KEYPIN, LOW);

	unsigned long long ullFreqHz = 0;
	si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);

	si5351.update_status();
	usleep(500000);

	printf("SYS_INIT: %d\n", si5351.dev_status.SYS_INIT);
	printf("LOL_A: %d\n", si5351.dev_status.LOL_A);
	printf("LOL_B: %d\n", si5351.dev_status.LOL_B);
	printf("LOS: %d\n", si5351.dev_status.LOS);
	printf("REVID: %d\n", si5351.dev_status.REVID);

	return(0);
}
