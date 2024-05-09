#include <stdio.h>
#include <wiringPi.h>

// RESETPIN is BCM pin 17 = physical pin 11
#define RESETPIN 17

int main(void)
{
  // uses BCM numbering of the GPIOs and directly accesses the GPIO registers.
  wiringPiSetupGpio();

 
  // Set pin mode (INPUT, OUTPUT, PWM_OUTPUT, GPIO_CLOCK)
  pinMode(RESETPIN, OUTPUT);

  digitalWrite (RESETPIN, LOW);
  delay(50);

  digitalWrite (RESETPIN, HIGH);
  delay(50);
	
  digitalWrite (RESETPIN, LOW);

  return 0;
}
