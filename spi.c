#include "spi.h"

//***********************************
//***********************************
//********** SPI OPEN PORT **********
//***********************************
//***********************************
//spi_device	0=CS0, 1=CS1
int SpiOpenPort (int spi_device, int *spi_cs_fd, unsigned char spi_mode, unsigned char spi_bitsPerWord, unsigned int spi_speed)
{
  int status_value = -1;
  
  if (spi_device)
    *spi_cs_fd = open("/dev/spidev0.1", O_RDWR);
  else
    *spi_cs_fd = open("/dev/spidev0.0", O_RDWR);

  if (*spi_cs_fd < 0)
    {
      perror("Error - Could not open SPI device");
      exit(1);
    }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MODE, &spi_mode);
  if(status_value < 0)
    {
      perror("Could not set SPIMode (WR)...ioctl fail");
      exit(1);
    }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MODE, &spi_mode);
  if(status_value < 0)
    {
      perror("Could not set SPIMode (RD)...ioctl fail");
      exit(1);
    }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
  if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      exit(1);
    }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
  if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      exit(1);
    }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if(status_value < 0)
    {
      perror("Could not set SPI speed (WR)...ioctl fail");
      exit(1);
    }

  status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if(status_value < 0)
    {
      perror("Could not set SPI speed (RD)...ioctl fail");
      exit(1);
    }
  return(status_value);
}



//************************************
//************************************
//********** SPI CLOSE PORT **********
//************************************
//************************************
int SpiClosePort (int spi_device, int *spi_cs_fd)
{
  int status_value = -1;

  status_value = close(*spi_cs_fd);
  if(status_value < 0)
    {
      perror("Error - Could not close SPI device");
      exit(1);
    }
  return(status_value);
}



//*******************************************
//*******************************************
//********** SPI WRITE & READ DATA **********
//*******************************************
//*******************************************
//SpiDevice		0 or 1
//TxData and RxData can be the same buffer (read of each byte occurs before write)
//Length		Max 511 (a C SPI limitation it seems)
//LeaveCsLow	1=Do not return CS high at end of transfer (you will be making a further call to transfer more data), 0=Set CS high when done
int SpiWriteAndRead (int *spi_cs_fd, unsigned char *TxData, unsigned char *RxData, int Length, int LeaveCsLow, unsigned char spi_bitsPerWord, unsigned int spi_speed)
{
  struct spi_ioc_transfer spi;
  int i = 0;
  int retVal = -1;

  spi.tx_buf = (unsigned long)TxData;		//transmit from "data"
  spi.rx_buf = (unsigned long)RxData;		//receive into "data"
  spi.len = Length;
  spi.delay_usecs = 0;
  spi.speed_hz = spi_speed;
  spi.bits_per_word = spi_bitsPerWord;
  spi.cs_change = LeaveCsLow;						//0=Set CS high after a transfer, 1=leave CS set low

  retVal = ioctl(*spi_cs_fd, SPI_IOC_MESSAGE(1), &spi);

  if(retVal < 0)
    {
      perror("Error - Problem transmitting spi data..ioctl");
      exit(1);
    }

  return retVal;
}
