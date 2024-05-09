#include <stdio.h>
#include <unistd.h>		// Used for UART
#include <fcntl.h>		// Used for UART
#include <termios.h>		// Used for UART
#include <stdlib.h>
#include <string.h>

int UARTOpenPort (int uart_filestream);

int main(int argc, char *argv[]) {
  int uart0_filestream;

  uart0_filestream = UARTOpenPort(uart0_filestream);

  unsigned char txbuffer[128] = "TX0;";
  int count;

  count = write(uart0_filestream, &txbuffer[0], strlen(txbuffer));
  if (count < 0) {
    perror("Error - Could not write to UART device");
    exit(1);
  }
  printf("Wrote %d bytes\n", strlen(txbuffer));
  
  close(uart0_filestream);
  return(0);
}
  
int UARTOpenPort (int uart_filestream) {
  uart_filestream = -1;
	
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
  uart_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);		// Open in non-blocking read/write mode
  if (uart_filestream == -1)
    {
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
  struct termios options;
  tcgetattr(uart_filestream, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(uart_filestream, TCIFLUSH);
  tcsetattr(uart_filestream, TCSANOW, &options);

  return(uart_filestream);
}
