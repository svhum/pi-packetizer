#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#define MCP3221_I2C_ADDR 0x4d

int selectDevice(int fd, int addr, char *name)
{
  int s;
  char str[128];

  s = ioctl(fd, I2C_SLAVE, addr);

  if (s == -1)
    {
      sprintf(str, "selectDevice for %s", name);
      perror(str);
    }

  return s;
}

uint8_t i2c_write(int fd, uint8_t addr, uint8_t data)
{
	unsigned char buf[256];
	buf[0] = addr;
	buf[1] = data;
	return write(fd, buf, 2);
}

uint8_t i2c_read(int fd) //, uint8_t addr)
{
	uint8_t reg_val[2];
	unsigned char buf[256];
	//buf[0] = addr;
	//write(fd, buf, 1);
	read(fd, &reg_val, 2);		// assume all read is always for a single byte
	printf("%d, %d\n", reg_val[0], reg_val[1]);
	return (int16_t)reg_val;
}

int main(void) {
  uint8_t reg_val = 0;	// assume I2C hardware is already connected and initialized
  int fd;
  int bytesread;
  uint16_t adcvalue;
	
  unsigned char buf[256];
  sprintf(buf, "/dev/i2c-1");		// fix I2C bus to 1 (can be changed later)

  if ((fd = open(buf, O_RDWR)) < 0)
    {
      // Open port for reading and writing
      fprintf(stderr, "Failed to open i2c bus /dev/i2c-1\n");
      exit(1);
    }

  // Initialise i2c bus
  selectDevice(fd, MCP3221_I2C_ADDR, "mcp3221");		// use default I2C address defined in .h file

  // Read sample
  adcvalue = i2c_read(fd);
  printf("%d\n", adcvalue);
  close(fd);
}
