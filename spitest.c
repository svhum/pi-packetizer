#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "spi.h"

#define NUM_BYTES 4

int main(int argc, char *argv[])
{
  int spi_cs0_fd;				//file descriptor for the SPI device
  //uint32_t data_to_send = 0x12345678;
  //uint32_t data_to_send = 00225 | (1 << 16);
  uint32_t data_to_send = 10000;
  uint32_t data_received;
  unsigned char spi_bitsPerWord = 8;
  unsigned int spi_speed = 1000000;   // 1 MHz

  // SPI MODES -----
  // SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
  // SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
  // SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
  // SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge

  // Incrementing ptr goes from most-significant byte to least significant.
  // Therefore we must decrement for data to appear in the correct order 
  // on the slave. Sending 0x12345678, debug LEDs should show 0x8.
  // Can experiment with the shift to confirm correct LED output.
  data_to_send = data_to_send >> 0;

  SpiOpenPort(0, &spi_cs0_fd, SPI_MODE_0, spi_bitsPerWord, spi_speed);
  usleep(1000);
  
  while (1) {
      data_received = SpiWriteAndRead4(&spi_cs0_fd, data_to_send, spi_bitsPerWord, spi_speed);
      printf("Received: 0x%x\n\n", data_received);
      usleep(50000);
  }

  SpiClosePort(0, &spi_cs0_fd);

  return(0);
}

