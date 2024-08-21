#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>		// Used for UART: write(), read(), close()
#include <fcntl.h>		// Used for UART: file controls like O_RDWR
#include <termios.h>		// Used for UART: Contains POSIX terminal control defns

/*
  Serial reference:
  https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/

// Constants
#define FSAMP 48000
#define FCORR 728
#define FCENT 10000
#define MAX_DF 12000

int OpenSerDev (const char *dev);
int writestr(int uart_filestream, const char *str, size_t bytes);

// Global variables
int tx = 0;
int mode = 2;			/* 0 = I/Q, 1 = LSB, 2 = USB, 12 = DUSB */

int main(int argc, char *argv[]) {
  // Serial port variables
  int serdev0_filestream;

  // CAT variables
  unsigned char cmdbuf_ch;
  unsigned char cmdbuf[32] = {0};
  unsigned char cmdbuf_ptr = 0;
  int count;
  unsigned char txbuffer[128];

  // Frequency variables
  unsigned long freq = 14074000;
  unsigned long freq_a = 14064000;
  unsigned long freq_b = FCENT+FCORR;
  unsigned long phasediff = freq_b*65536/FSAMP;
  char *ptr;
  
  // State variables
  unsigned long curr_cfg;
  unsigned char tempmode;
  unsigned char ai = 0;
  unsigned char st = 0;
  unsigned char id_str[6] = "0650;";
  unsigned char sh_str[8] = "SH0000;";
  unsigned char na_str[6] = "NA00;";

  if (argc != 2) {
    fprintf(stderr, "Usage: serial-test <serial device>\n");
    exit(1);
  }

  serdev0_filestream = OpenSerDev(argv[1]);

  while (1) {
    count = read(serdev0_filestream, &cmdbuf_ch, 1);
    if (count < 0) {
      perror("Error reading from serial device");
      exit(1);
    }
    //putchar(cmdbuf_ch); // Local echo
    
    if (cmdbuf_ptr < 32) {
      cmdbuf[cmdbuf_ptr++] = cmdbuf_ch;
      if (cmdbuf_ch == ';') { // Full command received
	cmdbuf_ptr = 0;
	//curr_cfg = XGpio_DiscreteRead(&sdr_cfg_reg, 1);
	if (memcmp(cmdbuf, "TX", 2) == 0) {
	  if (cmdbuf[2] == '0') {
	    tx = 0;
	    //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg & 0b011111111111111111);
	  }
	  else if (cmdbuf[2] == '1') {
	    tx = 1;
	    //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg | 0b100000000000000000);
	  }
	  else if (cmdbuf[2] == ';') {
	    sprintf(txbuffer, "TX%d;", tx);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	}
#if 1
	else if (memcmp(cmdbuf, "FA", 2) == 0) {
	  if (cmdbuf[2] == ';')
	    if (mode == 0) {
	      sprintf(txbuffer, "FA%09d;", freq_a);
	      writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	    }
	    else {
	      sprintf(txbuffer, "FA%09d;", freq);
	      writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	    }
	  else {
	    freq = strtol(&cmdbuf[2], &ptr, 10);
	    if (mode == 0) { // For IQ passthrough, do not set DDS explicitly
	      //set_LO_freq(freq);
	      freq_a = freq;
	    }
	    else { // For all other modes, apply DDC, adjusting LO and SDR based on frequency change
	      if ((freq - freq_a) > MAX_DF || (freq - freq_a) < 0) { // PLL and NCO need to change
		freq_a = freq - FCENT;
		freq_b = FCENT + FCORR;
		//set_LO_freq(freq_a);
		phasediff = freq_b*65536/FSAMP;
		//XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, (curr_cfg & 0b110000000000000000) | (phasediff & 0xffff));
	      }
	      else { // Only DDS needs to change
		freq_b = freq-freq_a+FCORR;
		phasediff = freq_b*65536/FSAMP;
		//XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, (curr_cfg & 0b110000000000000000) | (phasediff & 0xffff));
	      }
	    }
	  }
	}
#endif
	else if (memcmp(cmdbuf, "FB", 2) == 0) {
	  if (cmdbuf[2] == ';') {
	    sprintf(txbuffer, "FB%09d;", freq_b);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	  else {
	    freq_b = strtol(&cmdbuf[2], &ptr, 10);
	    phasediff = freq_b*65536/FSAMP;
	    //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, (curr_cfg & 0b110000000000000000) | (phasediff & 0xffff));
	  }
	}

	else if (memcmp(cmdbuf, "MD0", 3) == 0) {
	  if (cmdbuf[3] == ';') { // Query is MD0;
	    sprintf(txbuffer, "MD0%x;", mode);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	  else {
	    tempmode = strtol(&cmdbuf[3], &ptr, 16);
	    if (tempmode == 0) { // 0 = I/Q passthrough, not part of the CAT standard
	      mode = 0;
	      //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg | 0b010000000000000000);
	    }
	    else if (tempmode == 1) { // 1 = LSB; both USB and LSB use the same config bit since both go out the DAC
	      mode = 1;
	      //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg & 0b101111111111111111);
	    }
	    else if (tempmode == 2) { // 2 = USB 
	      mode = 2;
	      //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg & 0b101111111111111111);
	    }
	    else if (tempmode == 9) { // 9 = I/Q with DDC instead of RTTY-USB
	      mode = 9;
	      //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg | 0b010000000000000000);
	    }
	    else if (tempmode == 12) { // 12 = 0xc = DATA-USB
	      mode = 12;
	      //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg & 0b101111111111111111);
	    }
	  }
	}
	else if (memcmp(cmdbuf, "AI", 2) == 0) {
	  if (cmdbuf[2] == '0')
	    ai = 0;
	  else if (cmdbuf[2] == '1')
	    ai = 1;
	  else if (cmdbuf[2] == ';') {
	    sprintf(txbuffer, "AI%d;", ai);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	}
	else if (memcmp(cmdbuf, "ID", 2) == 0) {
	  if (cmdbuf[2] == ';') {
	    sprintf(txbuffer, "ID%s;", id_str);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	}
	else if (memcmp(cmdbuf, "SH0", 3) == 0) {
	  if (cmdbuf[3] == ';') {
	    sprintf(txbuffer, "%s;", sh_str);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	}
	else if (memcmp(cmdbuf, "NA0", 3) == 0) {
	  if (cmdbuf[3] == ';') {
	    sprintf(txbuffer, "%s;", na_str);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	}
	else if (memcmp(cmdbuf, "IF", 2) == 0) {
	  sprintf(txbuffer, "IF001%09d+000000%x00000;", freq, mode);
	  writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	}
	else if (memcmp(cmdbuf, "ST", 2) == 0) {
	  if (cmdbuf[2] == '0')
	    st = 0;
	  else if (cmdbuf[2] == '1')
	    st = 1;
	  else if (cmdbuf[2] == ';') {
	    sprintf(txbuffer, "ST%d;", st);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	}
      }
    }

  }
  
  close(serdev0_filestream);
  return(0);
}
  
int OpenSerDev(const char *dev) {
  int uart_filestream = -1;
  
  // OPEN THE UART
  // The flags (defined in fcntl.h):
  //	Access modes (use 1 of these):
  //		O_RDONLY - Open for reading only.
  //		O_RDWR - Open for reading and writing.
  //		O_WRONLY - Open for writing only.
  //
  //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
  //	if there is no input immediately available (instead of blocking). Likewise, write requests can also return
  //	immediately with a failure status if the output can't be written immediately.
  //
  //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
  //uart_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);		// Open in non-blocking read/write mode
  //uart_filestream = open("/dev/pts/12", O_RDWR | O_NOCTTY | O_NDELAY);		// Open in non-blocking read/write mode
  uart_filestream = open(dev, O_RDWR);		// Open in non-blocking read/write mode
  if (uart_filestream == -1) {
    perror("Error - Could not open UART device");
    exit(1);
  }
  
  // CONFIGURE THE UART
  // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
  //	CSIZE:- CS5, CS6, CS7, CS8
  //	CLOCAL - Ignore modem status lines
  //	CREAD - Enable receiver
  //	IGNPAR = Ignore characters with parity errors
  //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
  //	PARENB - Parity enable
  //	PARODD - Odd parity (else even)
  struct termios tty;

  // Read in current settings
  if (tcgetattr(uart_filestream, &tty) != 0) {
    perror("Error from tcgetattr");
    exit(1);
  }

  tty.c_cflag &= ~PARENB;	/* Clear parity bit, disabling parity */
  tty.c_cflag &= ~CSTOPB;       /* Clear stop field, only one stop bit used in communication */
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte 
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  // Blocking read for at least 1 character
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 1;
  
  /* options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; /\* Control mode flags *\/ */
  /* options.c_iflag = IGNPAR;			    /\* Input mode flags *\/ */
  /* options.c_oflag = 0;				    /\* Output mode flags *\/ */
  /* options.c_lflag = 0;				    /\* Local mode fpags *\/ */

  if (tcsetattr(uart_filestream, TCSANOW, &tty) != 0) {
    perror("Error setting attributes");
    exit(1);
  }
  tcflush(uart_filestream, TCIFLUSH);

  return(uart_filestream);
}

int writestr(int uart_filestream, const char *str, size_t bytes) {
  int count = write(uart_filestream, str, bytes);
  if (count < 0) {
    perror("Error writing to serial device");
    exit(1);
  }
  return(count);
}
