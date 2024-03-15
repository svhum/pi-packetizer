#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <termios.h>
#include <alsa/asoundlib.h>

#define NUM_IQ 504*4
//#define NUM_IQ 1008
//#define NUM_IQ 2032
//#define NUM_IQ 3048
//#define NUM_IQ 32256

/* volatile uint32_t *rx_freq; */
/* volatile uint16_t *rx_rate, *rx_cntr; */
/* volatile uint8_t *rx_rst; */
/* volatile uint8_t *rx_data; */
uint32_t rx_freq[8];
uint16_t rx_rate, rx_cntr;
uint8_t *rx_data;

const uint32_t freq_min = 0;
const uint32_t freq_max = 61440000;

int receivers = 1;

int sock_ep2;
struct sockaddr_in addr_ep6;

int enable_thread = 0;
int active_thread = 0;

snd_pcm_t *capture_handle;
int err = 0;

int uart0_filestream = -1;
unsigned char txbuffer[128];
uint32_t prevfreq = 0;

void process_ep2(uint8_t *frame);
void *handler_ep6(void *arg);

int init_soundcard(char *snd_device, snd_pcm_t **capture_handle, snd_pcm_hw_params_t **hw_params);

int UARTOpenPort (int uart_filestream);

int main(int argc, char *argv[])
{
  int fd, i;
  ssize_t size;
  pthread_t thread;
  /* volatile void *cfg, *sts; */
  /* volatile uint8_t *rx_sel; */
  uint8_t rx_sel;
  char *end;
  uint8_t buffer[1032];
  uint8_t reply[20] = {0xef, 0xfe, 2, 0, 0, 0, 0, 0, 0, 25, 1, 'R', '_', 'P', 'I', 'T', 'A', 'Y', 'A', 8};
  uint32_t code;
  struct ifreq hwaddr;
  struct sockaddr_in addr_ep2, addr_from;
  socklen_t size_from;
  int yes = 1;
  uint8_t chan = 0;
  long number;
  
  /* for(i = 0; i < 8; ++i) */
  /* { */
  /*   errno = 0; */
  /*   number = (argc == 10) ? strtol(argv[i + 2], &end, 10) : -1; */
  /*   if(errno != 0 || end == argv[i + 2] || number < 1 || number > 2) */
  /*   { */
  /*     fprintf(stderr, "Usage: sdr-receiver-hpsdr interface 1|2 1|2 1|2 1|2 1|2 1|2 1|2 1|2\n"); */
  /*     return EXIT_FAILURE; */
  /*   } */
  /*   // SVH: chan is copied to an 8-bit register selecting which receivers to enable from the CIC; not used */
  /*   chan |= (number - 1) << i; */
  /* } */

  errno = 0;
  if (argc != 2) {
    fprintf(stderr, "Usage: sdr-receiver-hpsdr interface\n");
    return EXIT_FAILURE;
  }

  /* if((fd = open("/dev/mem", O_RDWR)) < 0) */
  /* { */
  /*   perror("open"); */
  /*   return EXIT_FAILURE; */
  /* } */

  snd_pcm_hw_params_t *hw_params;

  //int err = 0;
  char *snd_device = "hw:1,1";
  if (err = init_soundcard(snd_device, &capture_handle, &hw_params)) {
    perror("initsound");
    return EXIT_FAILURE;
  }

  // Initialize UART
  uart0_filestream = UARTOpenPort(uart0_filestream);
  /* sprintf(txbuffer, "MD00;"); */
  /* int count = write(uart0_filestream, &txbuffer[0], strlen(txbuffer)); */
  /* if (count < 0) { */
  /*   perror("Error - Could not write to UART device"); */
  /*   exit(1); */
  /* } */
  
  // _SC_PAGESIZE is the page size in bytes (4096 on Intel), rx_data is 32 pages in size
  //rx_data = (uint8_t*) malloc(32*sysconf(_SC_PAGESIZE)*sizeof(uint8_t));
  rx_data = (uint8_t*) malloc(NUM_IQ*8*sizeof(uint8_t));
  if (rx_data == NULL) {
    perror("malloc");
    return EXIT_FAILURE;
  }

  /* rx_rst = (uint8_t *)(cfg + 0); */
  /* rx_sel = (uint8_t *)(cfg + 1); */
  /* rx_rate = (uint16_t *)(cfg + 2); */
  /* rx_freq = (uint32_t *)(cfg + 4); */

  /* rx_cntr = (uint16_t *)(sts + 0); */

  /* set default rx phase increment */
  for(i = 0; i < 8; ++i)
  {
    rx_freq[i] = (uint32_t)floor(600000 / 122.88e6 * (1 << 30) + 0.5);
  }

  /* set default rx sample rate */
  /* *rx_rate = 1280; */
  rx_rate = 1280;

  /* *rx_sel = chan; */
  rx_sel = chan;

  if((sock_ep2 = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket");
    return EXIT_FAILURE;
  }

  memset(&hwaddr, 0, sizeof(hwaddr));
  strncpy(hwaddr.ifr_name, argv[1], IFNAMSIZ - 1);
  ioctl(sock_ep2, SIOCGIFHWADDR, &hwaddr);
  for(i = 0; i < 6; ++i) reply[i + 3] = hwaddr.ifr_addr.sa_data[i];

  setsockopt(sock_ep2, SOL_SOCKET, SO_REUSEADDR, (void *)&yes , sizeof(yes));

  if(setsockopt(sock_ep2, SOL_SOCKET, SO_BINDTODEVICE, argv[1], strlen(argv[1]) + 1) < 0)
  {
    perror("SO_BINDTODEVICE");
    return EXIT_FAILURE;
  }

  memset(&addr_ep2, 0, sizeof(addr_ep2));
  addr_ep2.sin_family = AF_INET;
  addr_ep2.sin_addr.s_addr = htonl(INADDR_ANY);
  addr_ep2.sin_port = htons(1024);

  if(bind(sock_ep2, (struct sockaddr *)&addr_ep2, sizeof(addr_ep2)) < 0)
  {
    perror("bind");
    return EXIT_FAILURE;
  }

  while(1)
  {
    
    size_from = sizeof(addr_from);
    size = recvfrom(sock_ep2, buffer, 1032, 0, (struct sockaddr *)&addr_from, &size_from);
    if(size < 0)
    {
      perror("recvfrom");
      return EXIT_FAILURE;
    }

    memcpy(&code, buffer, 4);
    switch(code)
    {
      case 0x0201feef:
	// Payload is 11 bytes + 2 512-byte frames
	// 0xEFFE + 0x01 + 1 byte endpoint + 4 byte sequence number + 3 sync bytes = 11 bytes
        process_ep2(buffer + 11);
        process_ep2(buffer + 523);
        break;
      case 0x0002feef:
        reply[2] = 2 + active_thread;
        memset(buffer, 0, 60);
        memcpy(buffer, reply, 20);
        sendto(sock_ep2, buffer, 60, 0, (struct sockaddr *)&addr_from, size_from);
        break;
      case 0x0004feef:
        enable_thread = 0;
        while(active_thread) usleep(1000);
        break;
      case 0x0104feef:
      case 0x0204feef:
      case 0x0304feef:
	enable_thread = 0;
        while(active_thread) usleep(1000);

	//printf("Reading...\n");
	
	snd_pcm_uframes_t buffer_size;
	snd_pcm_uframes_t period_size;

	snd_pcm_get_params(capture_handle, &buffer_size, &period_size);
	printf("buffer size=%d, period size=%d\n", buffer_size, period_size);

	// Read NUM_IQ frames -- moved to thread
	/* if ((err = snd_pcm_readi(capture_handle, rx_data, NUM_IQ)) != NUM_IQ) { */
	/*   perror("read from audio interface failed"); */
	/*   if (err == -32) // Broken pipe */
	/*     { */
	/*       if (err = snd_pcm_prepare(capture_handle)) { */
	/* 	perror("cannot prepare audio interface for use"); */
	/* 	return(-1); */
	/*       } */
	/*     } */
	/*   else */
	/*     return(-1); */
	/* } */
	
        memset(&addr_ep6, 0, sizeof(addr_ep6));
        addr_ep6.sin_family = AF_INET;
        addr_ep6.sin_addr.s_addr = addr_from.sin_addr.s_addr;
        addr_ep6.sin_port = addr_from.sin_port;
        enable_thread = 1;
        active_thread = 1;
        if(pthread_create(&thread, NULL, handler_ep6, NULL) < 0)
        {
          perror("pthread_create");
          return EXIT_FAILURE;
        }
        pthread_detach(thread);
        break;
    }
  }

  close(sock_ep2);
  free(rx_data);
  snd_pcm_close(capture_handle);

  close(uart0_filestream);
  
  return EXIT_SUCCESS;
}

void process_ep2(uint8_t *frame)
{
  uint32_t freq;
  /* uint32_t freq, freq_div_100; */

  switch(frame[0])
  {
    case 0:
    case 1:
      receivers = ((frame[4] >> 3) & 7) + 1;

      /* set rx sample rate */
      switch(frame[1] & 3)
      {
        case 0:
          /* *rx_rate = 1280; */
          rx_rate = 1280;	/* 48 kS/s */
          break;
        case 1:
          /* *rx_rate = 640; */
          rx_rate = 640;	/* 96 kS/s */
	  /* printf("Set rate to 96\n"); */
          break;
        case 2:
          /* *rx_rate = 320; */
          rx_rate = 320;	/* 192 kS/s */
          break;
        case 3:
          /* *rx_rate = 160; */
          rx_rate = 160;	/* 384 kS/s */
          break;
      }
      break;
    case 4:
    case 5:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      //freq_div_100 = freq/100;
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[0] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      if (freq != prevfreq) {
	sprintf(txbuffer, "FA%d;", freq);
	/* printf("%s\n", txbuffer); */
	int count = write(uart0_filestream, &txbuffer[0], strlen(txbuffer));
	if (count < 0) {
	  perror("Error - Could not write to UART device");
	  exit(1);
	}
	prevfreq = freq;
      }
      break;
    case 6:
    case 7:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[1] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      break;
    case 8:
    case 9:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[2] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      break;
    case 10:
    case 11:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[3] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      break;
    case 12:
    case 13:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[4] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      break;
    case 14:
    case 15:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[5] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      break;
    case 16:
    case 17:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[6] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      break;
    case 36:
    case 37:
      /* set rx phase increment */
      freq = ntohl(*(uint32_t *)(frame + 1));
      if(freq < freq_min || freq > freq_max) break;
      rx_freq[7] = (uint32_t)floor(freq / 122.88e6 * (1 << 30) + 0.5);
      break;
  }
}

void *handler_ep6(void *arg)
{
  int i, j, n, m, p, size;
  int data_offset, header_offset;
  uint32_t counter;
  //uint8_t data[6 * 4096];
  uint8_t buffer[25 * 1032];	/* Metis data frame is 1024 + 8 = 1032 bytes; make buffer to hold 25 of them (25,800 bytes) */
  uint8_t *pointer;
  struct iovec iovec[25][1];
  struct mmsghdr datagram[25];
  uint8_t header[40] =
  {
    127, 127, 127, 0, 0, 33, 17, 25,
    127, 127, 127, 8, 0, 0, 0, 0,
    127, 127, 127, 16, 0, 0, 0, 0,
    127, 127, 127, 24, 0, 0, 0, 0,
    127, 127, 127, 32, 66, 66, 66, 66
  };
  
  memset(iovec, 0, sizeof(iovec));
  memset(datagram, 0, sizeof(datagram));

  // Fill in 4-byte 0x0601feef leader for 25 Metis data frames
  // Note: m frames are actually sent
  for(i = 0; i < 25; ++i)
  {
    *(uint32_t *)(buffer + i * 1032 + 0) = 0x0601feef; /* Endpoint 6, 0x01 (I&Q+M) */
    iovec[i][0].iov_base = buffer + i * 1032;
    iovec[i][0].iov_len = 1032;
    datagram[i].msg_hdr.msg_iov = iovec[i];
    datagram[i].msg_hdr.msg_iovlen = 1;
    datagram[i].msg_hdr.msg_name = &addr_ep6;
    datagram[i].msg_hdr.msg_namelen = sizeof(addr_ep6);
  }

  header_offset = 0;
  counter = 0;

  /* *rx_rst &= ~1; */
  /* *rx_rst |= 1; */

  while(1)
  {
    if(!enable_thread) break;

    /* size is the # of bytes in an IQ+M sample per RX: 3 for I, 3 for
       Q, 2 for mic data e.g. 8 bytes/sample for 1 receiver, 50
       bytes/sample for 8 receivers

       n is the number of IQ+M samples that will fit in a 504-byte
       payload (samples/payload) (512 bytes - 3 sync bytes - 5 C&C
       bytes = 504 bytes/payload). The rest is zero-padded. (10-63)

       m is the number of sequences to transmit, where a sequence
       contains the full EP6 frame (2 512 payloads and hence 2*n IQ+M
       samples).
       
       The expected buffer size is to be 2*m*n IQ samples or 2*m*n*p
       bytes, where p is the total number of *FIFO* bytes needed for
       an IQ sample from all the receivers, e.g. 8 bytes for two
       32-bit I/Q samples. The result is multiplied by 2 because there
       are 2 512-byte payloads in an EP6 packet. */
       
    size = receivers * 6 + 2;
    n = 504 / size;
    //m = 256 / n;
    m = NUM_IQ/(2*n);
    p = 8;
    //printf("receivers=%d, n=%d, m=%d, m*n*96=%d\n", receivers, n, m, m*n*96);

    /* if(*rx_cntr >= 2048) */
    /* { */
    /*   *rx_rst &= ~1; */
    /*   *rx_rst |= 1; */
    /* } */

    /* while(*rx_cntr < m * n * 2) usleep(1000); */
    
    //memcpy(data, rx_data, m * n * 96);
    //memcpy(data, rx_data, m * n * 2 * p);

    // Read NUM_IQ frames
    if ((err = snd_pcm_readi(capture_handle, rx_data, NUM_IQ)) != NUM_IQ) {
      perror("read from audio interface failed");
      if (err == -32) // Broken pipe
	{
	  if (err = snd_pcm_prepare(capture_handle)) {
	    perror("cannot prepare audio interface for use");
	    exit(-1);
	  }
	}
      else
	exit(-1);
    }
   
    data_offset = 0;

    // Add 4-byte (32-bit) sequence numbers to buffer.
    for(i = 0; i < m; ++i)
    {
      *(uint32_t *)(buffer + i * 1032 + 4) = htonl(counter);
      ++counter;
    }

    // Each EP6 frame is 512 bytes, of which 504 is data
    for(i = 0; i < m * 2; ++i)
    {
      // Write current header to first 8 bytes and select next 8-byte header
      pointer = buffer + i * 516 - i % 2 * 4 + 8;
      memcpy(pointer, header + header_offset, 8);
      header_offset = header_offset >= 32 ? 0 : header_offset + 8;

      // Advance pointer to payload position and clear payload area
      pointer += 8;
      memset(pointer, 0, 504);

      // For each IQ+M sample in the 504-byte data block, copy data in
      // Note that rx_data is LE but the buffer is expecting network byte order
      for(j = 0; j < n; ++j)
      {
	// Write most-significant 24 bits of I to buffer
        memcpy(pointer+3, rx_data + data_offset+3, 1);
        memcpy(pointer+4, rx_data + data_offset+2, 1);
        memcpy(pointer+5, rx_data + data_offset+1, 1);
	//data_offset += p/2;	/* Advance to Q */
	//pointer += 3;		/* Advance buffer pointer 3 bytes */
	
        memcpy(pointer, rx_data + data_offset+3+p/2, 1);
        memcpy(pointer+1, rx_data + data_offset+2+p/2, 1);
        memcpy(pointer+2, rx_data + data_offset+1+p/2, 1);
	//data_offset += p/2;	/* Advance to I */
	//pointer += 5;		/* Advance buffer pointer 5 bytes (skip mic data) */

	data_offset += p;	/* Advance buffer pointer to next I/Qsample */
	pointer += 8;		/* Advance buffer pointer 3+3+2 bytes (skip mic data) */
      }
    }

    sendmmsg(sock_ep2, datagram, m, 0);
  }

  active_thread = 0;

  return NULL;
}

int init_soundcard(char *snd_device, snd_pcm_t **capture_handle, snd_pcm_hw_params_t **hw_params) {
  int err = 0;
  
  if (err = snd_pcm_open(&*capture_handle, snd_device, SND_PCM_STREAM_CAPTURE, 0)) {
    perror("cannot open audio device");
    return(-1);
  }

  if (err = snd_pcm_hw_params_malloc(&*hw_params) < 0) {
    perror("cannot allocate hardware parameter structure");
    return(-1);
  }

  if (err = snd_pcm_hw_params_any(*capture_handle, *hw_params)) {
    perror("cannot initialize hardware parameter structure");
    return(-1);
  }

  if (err = snd_pcm_hw_params_set_access(*capture_handle, *hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) {
    perror("cannot set access type");
    return(-1);
  }

  if (err = snd_pcm_hw_params_set_format(*capture_handle, *hw_params, SND_PCM_FORMAT_S32_LE)) {
    perror("cannot set sample format");
    return(-1);
  }

  signed int srate = 96000;
  if (err = snd_pcm_hw_params_set_rate_near(*capture_handle, *hw_params, &srate, 0)) {
    perror("cannot set sample rate");
    return(-1);
  }

  if (err = snd_pcm_hw_params_set_channels(*capture_handle, *hw_params, 2)) {
    perror("cannot set channel count");
    return(-1);
  }

  snd_pcm_uframes_t period = NUM_IQ;
  int dir = 0;
  if (err = snd_pcm_hw_params_set_period_size_near(*capture_handle, *hw_params, &period, &dir)) {
     perror("cannot set period");
    return(-1);
  }

  if (err = snd_pcm_hw_params(*capture_handle, *hw_params)) {
    perror("cannot set parameters");
    return(-1);
  }

  if (err = snd_pcm_prepare(*capture_handle)) {
    perror("cannot prepare audio interface for use");
    return(-1);
  }

  if (err = snd_pcm_start(*capture_handle)) {
    perror("cannot start soundcard");
    return(-1);
  }
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
