#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <alsa/asoundlib.h>
#include "si5351.h"

#define NUM_IQ 504
#define SI5351_I2C_ADDRESS 0x60
#define SAMPRATE 48000

Si5351 si5351(SI5351_I2C_ADDRESS);

int init_soundcard(char *snd_device, snd_pcm_t **capture_handle, snd_pcm_hw_params_t **hw_params);

int main(int argc, char *argv[])
{
  /* FILE *fptr; */
  
  /* fptr = fopen("test.txt", "r"); */
  /* perror("file error"); */
  /* return(-1); */

  int err = 0;
  int control_si5351 = 0;     // Control Si5351 frequency based on measured frequency if set
  
  if (argc != 2) {
    fprintf(stderr, "Usage: packetizer soundcard-interface\n");
    return EXIT_FAILURE;
  }

  if (control_si5351)
    si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  

  char *snd_device = argv[1];
  printf("# Device: %s\n", snd_device);
  
  snd_pcm_t *capture_handle;
  snd_pcm_hw_params_t *hw_params;
  
  if (err = init_soundcard(snd_device, &capture_handle, &hw_params)) {
    perror("initsound");
    return EXIT_FAILURE;
  }

  snd_pcm_uframes_t buffer_size;
  snd_pcm_uframes_t period_size;
  
  snd_pcm_get_params(capture_handle, &buffer_size, &period_size);
  printf("# buffer size=%d, period size=%d\n", buffer_size, period_size);
  
  // Read MAX_BUF_SIZE 2-channel frames from card; each frame is 2 16-bit samples
  //char wav_data[MAX_BUF_SIZE * 4];
  int32_t wav_data[NUM_IQ * 2];	/* L+R */

  while (1) { 
  
    if ((err = snd_pcm_readi(capture_handle, wav_data, NUM_IQ)) != NUM_IQ) {
      perror("read from audio interface failed");
      if (err == -32) // Broken pipe
	{
	  if (err = snd_pcm_prepare(capture_handle)) {
	    perror("cannot prepare audio interface for use");
	    return(-1);
	  }
	}
      else
	return(-1);
    }

#ifdef DEBUG
    for (int k=0; k < NUM_IQ*2; k+=2) {
      //printf("%f\n", wav_data[k]/pow(2, 16));
      printf("%d\t%d\n", wav_data[k], wav_data[k+1]);
    }
#endif

	int k = 0, N = 0, found, eof = 0;
    double dt = 1.0/SAMPRATE;
	double m, dx, zA, zB, T, Tavg = 0, favg, favg_cHz;
	unsigned long long ullFreqHz = 0;	
	unsigned long long ullFreqHz_tune;	

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
		ullFreqHz = 0;
	}
	else {
  		Tavg /= N;
		favg = 1/Tavg;
		favg_cHz = round(favg * 100);
		ullFreqHz = (unsigned long long)(favg_cHz);
	}
  ullFreqHz_tune = ullFreqHz + 1407400000;
  if (control_si5351)
    si5351.set_freq(ullFreqHz_tune, SI5351_CLK0);
  printf("# Tavg = %e, favg = %.20e %llu %llu\n", Tavg, favg, ullFreqHz, ullFreqHz_tune);
  }
  
  snd_pcm_close(capture_handle);
  snd_pcm_hw_params_free(hw_params);
  
  return(0);
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

  unsigned int srate = SAMPRATE;
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

  return(0);
}
