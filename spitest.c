#include <fcntl.h>				//Needed for SPI port
#include <sys/ioctl.h>			//Needed for SPI port
#include <linux/spi/spidev.h>	//Needed for SPI port
#include <unistd.h>			//Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
/* #include <string> */
/* #include <iostream> */
/* #include <unistd.h> */
/* #include <cstring> */
#include <time.h>

#define NUM_BYTES 200

int spi_cs0_fd;				//file descriptor for the SPI device
int spi_cs1_fd;				//file descriptor for the SPI device
unsigned char spi_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;

int SpiOpenPort (int spi_device);
int SpiClosePort (int spi_device);
int SpiWriteAndRead (int SpiDevice, unsigned char *TxData, unsigned char *RxData, int Length, int LeaveCsLow);

int main(int argc, char *argv[])
{
  unsigned char rxdata[NUM_BYTES];
  unsigned char txdata[NUM_BYTES];
  time_t start, stop;
  double diff;

  for (int k=0; k < NUM_BYTES; k++)
    txdata[k] = k;
  
  SpiOpenPort(0);
  start = time(NULL);
  SpiWriteAndRead(0, &txdata[0], &rxdata[0], NUM_BYTES, 0);
  stop = time(NULL);
  diff = difftime(stop, start);
  //SpiWriteAndRead(0, "D", &rxdata, 1, 0);

  for (int k=0; k < 2; k++)
    printf("Received: %x\n", rxdata[k]);
  printf("Took %e sec\n", diff);
  SpiClosePort(0);

  return(0);
}

//***********************************
//***********************************
//********** SPI OPEN PORT **********
//***********************************
//***********************************
//spi_device	0=CS0, 1=CS1
int SpiOpenPort (int spi_device)
{
  int status_value = -1;
  int *spi_cs_fd;


  //----- SET SPI MODE -----
  //SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
  //SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
  //SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
  //SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
  spi_mode = SPI_MODE_1;
    
  //----- SET BITS PER WORD -----
  spi_bitsPerWord = 8;
    
  //----- SET SPI BUS SPEED -----
  //spi_speed = 1000000;		//1000000 = 1MHz (1uS per bit) 
  spi_speed = 40000000;		//1000000 = 1MHz (1uS per bit) 


  if (spi_device)
    spi_cs_fd = &spi_cs1_fd;
  else
    spi_cs_fd = &spi_cs0_fd;


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
int SpiClosePort (int spi_device)
{
  int status_value = -1;
  int *spi_cs_fd;

  if (spi_device)
    spi_cs_fd = &spi_cs1_fd;
  else
    spi_cs_fd = &spi_cs0_fd;


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
int SpiWriteAndRead (int SpiDevice, unsigned char *TxData, unsigned char *RxData, int Length, int LeaveCsLow)
{
  struct spi_ioc_transfer spi;
  int i = 0;
  int retVal = -1;
  int spi_cs_fd;

  if (SpiDevice)
    spi_cs_fd = spi_cs1_fd;
  else
    spi_cs_fd = spi_cs0_fd;

  spi.tx_buf = (unsigned long)TxData;		//transmit from "data"
  spi.rx_buf = (unsigned long)RxData;		//receive into "data"
  spi.len = Length;
  spi.delay_usecs = 0;
  spi.speed_hz = spi_speed;
  spi.bits_per_word = spi_bitsPerWord;
  spi.cs_change = LeaveCsLow;						//0=Set CS high after a transfer, 1=leave CS set low

  retVal = ioctl(spi_cs_fd, SPI_IOC_MESSAGE(1), &spi);

  if(retVal < 0)
    {
      perror("Error - Problem transmitting spi data..ioctl");
      exit(1);
    }

  return retVal;
}
