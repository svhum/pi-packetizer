#ifndef SPI_H_
#define SPI_H_

#include <fcntl.h>				    // Needed for SPI port
#include <sys/ioctl.h>			  // Needed for SPI port
#include <linux/spi/spidev.h>	// Needed for SPI port
#include <unistd.h>			      // Needed for SPI port
#include <stdint.h>           // Needed for data types
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

int SpiOpenPort (int spi_device, int *spi_cs_fd, unsigned char spi_mode, unsigned char spi_bitsPerWord, unsigned int spi_speed);
int SpiClosePort (int spi_device, int *spi_cs_fd);
int SpiWriteAndRead (int *spi_cs_fd, unsigned char *TxData, unsigned char *RxData, int Length, int LeaveCsLow, unsigned char spi_bitsPerWord, unsigned int spi_speed);
uint32_t SpiWriteAndRead4 (int *spi_cs_fd, uint32_t data_to_send, unsigned char spi_bitsPerWord, unsigned int spi_speed);
#endif
