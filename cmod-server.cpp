#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>		// Used for UART: write(), read(), close()
#include <fcntl.h>		// Used for UART: file controls like O_RDWR
#include <termios.h>		// Used for UART: Contains POSIX terminal control defns
#include <pthread.h>     
#include <alsa/asoundlib.h>
#include <math.h>
#include <wiringPi.h>
#include "si5351.h"
#include "spi.h"

/*
  Serial reference:
  https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/

// Constants
#define FSAMP 48000
// Do not set FCORR=728, phasediff will not work with SSB modulator IP?
#define FCORR 0
#define FCENT 10000
#define MAX_DF 12000
#define SI5351_I2C_ADDRESS 0x60
#define NUM_IQ 504
#define MAX_SILENT_COUNT 2
#define SAMPRATE 48000
#define RESETPIN 17   // RESET is BCM pin 17 = physical pin 11
#define nTXENPIN 23   // nTXEN is BCM pin 23 = physical pin 16

#ifdef __arm__
Si5351 si5351(SI5351_I2C_ADDRESS);
#endif

int OpenSerDev (const char *dev);
int writestr(int uart_filestream, const char *str, size_t bytes);

void set_RX_freq(uint64_t freq_Hz);
void set_TX_freq(uint64_t freq_cHz);

int init_soundcard(char *snd_device, snd_pcm_t **capture_handle, snd_pcm_hw_params_t **hw_params);
uint64_t meas_freq_cHz(void);

void reset_cmod(void);

// Thread function
void* fsktx(void* data);

// Global variables
int tx = 0;
int mode = 2;			/* 0 = I/Q, 1 = LSB, 2 = USB, 12 = DUSB */
unsigned long freq = 14074000;
unsigned long freq_a = 14064000;
int poll_pty = 1;
uint32_t curr_cfg = FCENT;	// RX/TX = 0, SSB/IQ = 0 (receive SSB)

snd_pcm_t *capture_handle;
snd_pcm_hw_params_t *hw_params;
snd_pcm_uframes_t buffer_size;
snd_pcm_uframes_t period_size;

int main(int argc, char *argv[]) {
  // Behaviour variables
  int rx_only;		// If set, TX frequency is not modulated by server
  
  // Serial port variables
  int serdev0_filestream;

  // CAT variables
  unsigned char cmdbuf_ch;
  unsigned char cmdbuf[32] = {0};
  unsigned char cmdbuf_ptr = 0;
  int count;
  unsigned char txbuffer[128];

  // Frequency variables
  uint16_t freq_b = FCENT+FCORR;
  uint16_t phasediff = freq_b*65536/FSAMP;
  char *ptr;
  //unsigned long long ullFreq_cHz = freq_a*100;	
  
  // State variables
  unsigned char tempmode;
  unsigned char ai = 0;
  unsigned char st = 0;
  unsigned char id_str[6] = "0650;";
  unsigned char sh_str[8] = "SH0000;";
  unsigned char na_str[6] = "NA00;";

  // Thread variables
  pthread_t  thread_id;

  // ALSA variables
  int err;
  char *snd_device;

  // SPI variables
  int spi_cs0_fd;				// File descriptor for the SPI device
  unsigned char spi_bitsPerWord = 8;
  unsigned int spi_speed = 1000000;   // 1 MHz
  
  if (argc != 3 && argc != 2) {
    fprintf(stderr, "Usage: cmod-server <serial device> [ALSA device]\n");
    fprintf(stderr, "  If only <serial device> is given, the server will not\n");
    fprintf(stderr, "  modulate the Si5351 frequency during transmit cycles.\n");
    exit(1);
  }
  char *serial_device = argv[1];

  if (argc == 3) {
    snd_device = argv[2];
    rx_only = 0;
    printf("Listening on serial device %s and ALSA device %s...\n", serial_device,
	   snd_device);
  }
  else { // Only serial device given
    rx_only = 1;
    printf("No TX modulation will be used.\n");
    printf("Listening on serial device %s...\n", serial_device);
  }
  
  // Open serial device
  serdev0_filestream = OpenSerDev(serial_device);

#ifdef __arm__
  // Initialize and set Si5351
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  sleep(1);
  set_RX_freq(freq_a);
  si5351.output_enable(SI5351_CLK0, 1); // Not really required at power-on
  si5351.output_enable(SI5351_CLK1, 1); // Not really required at power-on
#endif

  // Initialize GPIO and reset cmod
  // Uses BCM numbering of the GPIOs and directly accesses the GPIO registers
  wiringPiSetupGpio();
  pinMode(RESETPIN, OUTPUT);
  pinMode(nTXENPIN, OUTPUT);
  reset_cmod();

  if (!rx_only) {
    // Initialize ALSA
    if (err = init_soundcard(snd_device, &capture_handle, &hw_params)) {
      perror("Error initializing sound card");
      exit(1);
    }
    if (err = snd_pcm_get_params(capture_handle, &buffer_size, &period_size)) {
      perror("Error setting PCM parameters");
      exit(1);
    }
    printf("# buffer size=%d, period size=%d\n", buffer_size, period_size);
 
    // Initiate TX modulator thread if needed
    if (pthread_create(&thread_id, NULL, fsktx, NULL) < 0) {
      perror("Error creating thread");
      exit(1);
    }
    printf("Created new thread (%u)... \n", thread_id);
  }

  // Initialize SPI and set control register: SSB, FCENT+FCORR
  SpiOpenPort(0, &spi_cs0_fd, SPI_MODE_0, spi_bitsPerWord, spi_speed);
  usleep(1000);
  SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
  usleep(1000);

  while (poll_pty) {
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
	if (memcmp(cmdbuf, "TX", 2) == 0) {
	  if (cmdbuf[2] == '0') {
	    tx = 0;
	    digitalWrite (nTXENPIN, HIGH);
	    curr_cfg = curr_cfg & 0x1ffff;
	    SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	  }
	  else if (cmdbuf[2] == '1') {
	    tx = 1;
	    digitalWrite (nTXENPIN, LOW);
	    curr_cfg = (curr_cfg & 0x1ffff) | 0x20000;
	    SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	  }
	  else if (cmdbuf[2] == ';') {
	    sprintf(txbuffer, "TX%d;", tx);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	}
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
  	      set_RX_freq(freq);
	      freq_a = freq;
	    }
	    else { // For all other modes, apply DDC, adjusting LO and SDR based on frequency change
	      if ((freq - freq_a) > MAX_DF || (freq - freq_a) < 0) { // PLL and NCO need to change
		freq_a = freq - FCENT;
		freq_b = FCENT + FCORR;
		set_RX_freq(freq_a);
		phasediff = freq_b*65536/FSAMP;
		curr_cfg = (curr_cfg & 0x30000) | (phasediff & 0xffff);
		SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	      }
	      else { // Only DDS needs to change
		freq_b = freq-freq_a+FCORR;
		phasediff = freq_b*65536/FSAMP;
		curr_cfg = (curr_cfg & 0x30000) | (phasediff & 0xffff);
		SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	      }
	    }
	  }
	}
	else if (memcmp(cmdbuf, "FB", 2) == 0) {
	  if (cmdbuf[2] == ';') {
	    sprintf(txbuffer, "FB%09d;", freq_b);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	  else {
	    freq_b = strtol(&cmdbuf[2], &ptr, 10);
	    phasediff = freq_b*65536/FSAMP;
	    curr_cfg = (curr_cfg & 0x30000) | (phasediff & 0xffff);
	    SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	  }
	}

	// Note on modes: 0 and 9 essentially pass I/Q through to a DDC. The intent was
	// that if the DDC is set with a 0 frequency LO (freq_b=0) then passthrough occurs,
	// while mode 9 would retain a nonzero DDC frequency, and potentially due some
	// filtering as well. But for now these modes behave identically.
	else if (memcmp(cmdbuf, "MD0", 3) == 0) {
	  if (cmdbuf[3] == ';') { // Query is MD0;
	    sprintf(txbuffer, "MD0%x;", mode);
	    writestr(serdev0_filestream, &txbuffer[0], strlen(txbuffer));
	  }
	  else {
	    tempmode = strtol(&cmdbuf[3], &ptr, 16);
	    if (tempmode == 0) { // 0 = I/Q passthrough, not part of the CAT standard
	      mode = 0;
	      curr_cfg = (curr_cfg & 0x2ffff) | 0x10000;
	      SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	    }
	    else if (tempmode == 1) { // 1 = LSB; both USB and LSB use the same config bit since both go out the DAC
	      mode = 1;
	      curr_cfg = curr_cfg & 0x2ffff;
	      SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	      //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg & 0b101111111111111111);
	    }
	    else if (tempmode == 2) { // 2 = USB 
	      mode = 2;
	      curr_cfg = curr_cfg & 0x2ffff;
	      SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	      //XGpio_DiscreteWrite(&sdr_cfg_reg, GPIO_CH, curr_cfg & 0b101111111111111111);
	    }
	    else if (tempmode == 9) { // 9 = I/Q with DDC instead of RTTY-USB;
	      mode = 9;
	      curr_cfg = (curr_cfg & 0x2ffff) | 0x10000;
	      SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
	    }
	    else if (tempmode == 12) { // 12 = 0xc = DATA-USB
	      mode = 12;
	      curr_cfg = curr_cfg & 0x2ffff;
	      SpiWriteAndRead4(&spi_cs0_fd, curr_cfg, spi_bitsPerWord, spi_speed);
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
	else if (memcmp(cmdbuf, "EX", 2) == 0) {
	  // Exit polling loop
	  poll_pty = 0;
	}
      }
    }

  }

  if (!rx_only) {
    snd_pcm_close(capture_handle);
    snd_pcm_hw_params_free(hw_params);
  }
  close(serdev0_filestream);
  SpiClosePort(0, &spi_cs0_fd);

  printf("Exiting main thread\n");
  pthread_exit(NULL);		/* terminate the thread */
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

void set_RX_freq(uint64_t freq_Hz) {
  uint8_t pll_div;
  uint64_t freq_cHz = freq_Hz*100;
  if (freq_Hz >= 6000000 && freq_Hz < 7500000)
    pll_div = 100;
  else if (freq_Hz >= 7500000 && freq_Hz < 10000000)
    pll_div = 80;
  else if (freq_Hz >= 10000000 && freq_Hz < 15000000)
    pll_div = 60; 
  else if (freq_Hz >= 15000000 && freq_Hz < 22500000)
    pll_div = 40;
  else
    return;

#ifdef __arm__
  si5351.set_freq_manual(freq_cHz, freq_cHz*pll_div, SI5351_CLK0);
  si5351.set_freq_manual(freq_cHz, freq_cHz*pll_div, SI5351_CLK1);
  //si5351.set_clock_invert(SI5351_CLK1, 0);
  //si5351.set_clock_invert(SI5351_CLK1, 0);
  si5351.set_phase(SI5351_CLK0, 0); 
  si5351.set_phase(SI5351_CLK1, pll_div);
  si5351.pll_reset(SI5351_PLLA);
#endif
  return;
}

void set_TX_freq(uint64_t freq_cHz) {
  uint8_t pll_div;
  if (freq_cHz >= 600000000 && freq_cHz < 750000000)
    pll_div = 100;
  else if (freq_cHz >= 750000000 && freq_cHz < 1000000000)
    pll_div = 80;
  else if (freq_cHz >= 1000000000 && freq_cHz < 1500000000)
    pll_div = 60; 
  else if (freq_cHz >= 1500000000 && freq_cHz < 2250000000)
    pll_div = 40;
  else
    return;

#ifdef __arm__
  //si5351.set_freq_manual(freq_cHz, freq_cHz*pll_div, SI5351_CLK0);
  //si5351.set_freq_manual(freq_cHz, freq_cHz*pll_div, SI5351_CLK1);
  //si5351.set_phase(SI5351_CLK0, 0); 
  //si5351.set_phase(SI5351_CLK1, 2*pll_div);

  // Restore phase shift to 0 and then invert outputs
  si5351.set_phase(SI5351_CLK0, 0); 
  si5351.set_phase(SI5351_CLK1, 0);
  si5351.pll_reset(SI5351_PLLA);
  si5351.set_clock_invert(SI5351_CLK0, 0);
  si5351.set_clock_invert(SI5351_CLK1, 1);

  // Set TX frequency
  si5351.set_freq(freq_cHz, SI5351_CLK0);
  si5351.set_freq(freq_cHz, SI5351_CLK1);
#endif
  return;
}

void* fsktx(void* data)
{
  uint64_t ullFreqcHz_tune, ullFreqcHz;	
  int outputs_enabled = 1, entered_tx = 0;
  int silent_count;

  // Initiate thread
  pthread_detach(pthread_self());
  
  while (poll_pty) {
    if (tx == 1) {
      // Set flag that transmission frequency was changed
      if (entered_tx == 0) {
	entered_tx = 1;
	printf("Entering TX mode, outputs_enabled = %d\n", outputs_enabled);
      }
     
      ullFreqcHz = meas_freq_cHz();

      if (ullFreqcHz == 0) {
	silent_count++;
	// Disable Si5351 output
	if (outputs_enabled == 1 && silent_count > MAX_SILENT_COUNT) {
	  printf("Silent message signal, disabling PLL outputs\n");
	  outputs_enabled = 0;
#ifdef __arm__
	  si5351.output_enable(SI5351_CLK0, 0);
	  si5351.output_enable(SI5351_CLK1, 0);
#endif
	}
      }
      else { // Message present on audio
	silent_count = 0;
	ullFreqcHz_tune = ullFreqcHz + freq*100;
#ifdef __arm__
	set_TX_freq(ullFreqcHz_tune);
#endif
	if (outputs_enabled == 0) {
#ifdef __arm__
	  si5351.output_enable(SI5351_CLK0, 1);
	  si5351.output_enable(SI5351_CLK1, 1);
#endif
	  outputs_enabled = 1;
	}

	//printf("# Tavg = %e, favg = %.20e %llu %llu\n", Tavg, favg, ullFreqcHz, ullFreqcHz_tune);
	printf("# freq = %llu, freq_tune = %llu\n", ullFreqcHz, ullFreqcHz_tune);
      }

    }
    else { 			// tx == 0, we are in receive mode
      if (entered_tx == 1) {
	// We previously were transmitting and now going back to receive mode.
	// Restore RX frequency and phase.
	printf("Returning to RX mode\n");
	entered_tx = 0;
#ifdef __arm__
	if (mode == 0)  // For IQ passthrough, do not set DDS explicitly
	  set_RX_freq(freq);
	else
	  set_RX_freq(freq_a);
#endif
      }
      if (outputs_enabled == 0) {
	// Outputs might have been previously disabled by a silent TX message signal; re-enable
	printf("Re-enabling PLL outputs\n");
	outputs_enabled = 1;
#ifdef __arm__
	si5351.output_enable(SI5351_CLK0, 1);
	si5351.output_enable(SI5351_CLK1, 1);
#endif
      }
      usleep(1000);
    }
  }

  printf("Closing frequency measurement thread\n");
  pthread_exit(NULL);			/* terminate the thread */
}

uint64_t meas_freq_cHz(void) {

  int err;
  int k = 0, N = 0, found, eof = 0;
  double dt = 1.0/SAMPRATE;
  double m, dx, zA, zB, T, Tavg = 0, favg, favg_cHz;
  uint64_t ullFreqcHz = 0;	
  int32_t wav_data[NUM_IQ * 2];	/* L+R */

  // Read from sound card
  if ((err = snd_pcm_readi(capture_handle, wav_data, NUM_IQ)) != NUM_IQ) {
    fprintf(stderr, "err = %d\n", err);
    perror("Read from audio interface failed");
    if (err == -32) // Broken pipe
      {
	if (err = snd_pcm_prepare(capture_handle)) {
	  perror("Cannot prepare audio interface for use");
	  exit(1);
	}
      }
    else
      exit(1);
  }

  // Find period
  while (!eof) {
    // Find first positive-going zero crossing
    found = 0;
    while (!found && !eof) {
      if (wav_data[k+2] >= 0 && wav_data[k] < 0)
	found = 1;
      else { 
	if (k+4 < NUM_IQ*2)
	  k += 2;
	else {
	  //printf("EOF1: k = %d\n", k);
	  eof = 1;
	}
      }
    }
    //printf("# First zero crossing @ %d: (%d, %d)\n", k/2, wav_data[k]/2, wav_data[k+2]/2);
    if (!eof) {	
      m = (wav_data[k+2] - wav_data[k]) / dt;
      dx = -wav_data[k]/m;
      zA = k/2*dt+dx;
    }

    // Find next positive-going zero crossing
    found = 0;
    k += 2;
    while (!found && !eof) {
      if (wav_data[k+2] >= 0 && wav_data[k] < 0)
	found = 1;
      else {
	if (k+4 < NUM_IQ*2)
	  k += 2;
	else {
	  //printf("EOF2: k = %d\n\n", k);
	  eof = 1;
	}
      }	
    }
    //printf("# Second zero crossing @ %d: (%d, %d)\n", k/2, wav_data[k]/2, wav_data[k+2]/2);
    if (!eof) {
      m = (wav_data[k+2] - wav_data[k]) / dt;
      dx = -wav_data[k]/m;
      zB = k/2*dt+dx;
      T = zB-zA;
      Tavg += T;
      N += 1;
    }

    if (!eof && 0)
      printf("# k = %4d, T = %e, f = %e\n", k, T, 1/T);

    k += 2;
  }
  if (N == 0) {
    Tavg = 0;
    favg = 0;
    ullFreqcHz = 0;
  }
  else {
    Tavg /= N;
    favg = 1/Tavg;
    favg_cHz = round(favg * 100);
    ullFreqcHz = (unsigned long long)(favg_cHz);
  }

  return(ullFreqcHz);
}

int init_soundcard(char *snd_device, snd_pcm_t **capture_handle, snd_pcm_hw_params_t **hw_params) {
  int err = 0;
  
  if (err = snd_pcm_open(&*capture_handle, snd_device, SND_PCM_STREAM_CAPTURE, 0)) {
    perror("Cannot open audio device");
    return(-1);
  }

  if (err = snd_pcm_hw_params_malloc(&*hw_params) < 0) {
    perror("Cannot allocate hardware parameter structure");
    return(-1);
  }

  if (err = snd_pcm_hw_params_any(*capture_handle, *hw_params)) {
    perror("Cannot initialize hardware parameter structure");
    return(-1);
  }

  if (err = snd_pcm_hw_params_set_access(*capture_handle, *hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) {
    perror("Cannot set access type");
    return(-1);
  }

  if (err = snd_pcm_hw_params_set_format(*capture_handle, *hw_params, SND_PCM_FORMAT_S32_LE)) {
    perror("Cannot set sample format");
    return(-1);
  }

  unsigned int srate = SAMPRATE;
  if (err = snd_pcm_hw_params_set_rate_near(*capture_handle, *hw_params, &srate, 0)) {
    perror("Cannot set sample rate");
    return(-1);
  }

  if (err = snd_pcm_hw_params_set_channels(*capture_handle, *hw_params, 2)) {
    perror("Cannot set channel count");
    return(-1);
  }

  snd_pcm_uframes_t period = NUM_IQ;
  int dir = 0;
  if (err = snd_pcm_hw_params_set_period_size_near(*capture_handle, *hw_params, &period, &dir)) {
    perror("Cannot set period");
    return(-1);
  }
 
  if (err = snd_pcm_hw_params(*capture_handle, *hw_params)) {
    perror("Cannot set parameters");
    return(-1);
  }

  if (err = snd_pcm_prepare(*capture_handle)) {
    perror("Cannot prepare audio interface for use");
    return(-1);
  }

  if (err = snd_pcm_start(*capture_handle)) {
    perror("Cannot start soundcard");
    return(-1);
  }

  return(0);
}

void reset_cmod(void) {
  // Set pin mode (INPUT, OUTPUT, PWM_OUTPUT, GPIO_CLOCK)
  digitalWrite (nTXENPIN, HIGH);
  digitalWrite (RESETPIN, LOW);
  delay(50);

  digitalWrite (RESETPIN, HIGH);
  delay(50);
	
  digitalWrite (RESETPIN, LOW);
}
