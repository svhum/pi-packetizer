/*==========================================================================

  tonegen 
  program.c
  Copyright (c)2020 Kevin Boone
  Distributed under the terms of the GPL v3.0

  This file contains the main body of the program. By the time
  program_run() has been called, RC files will have been read and comand-
  line arguments parsed, so all the contextual information will be in the
  ProgramContext. Logging will have been initialized, so the log_xxx
  methods will work, and be filtered at the appopriate levels.

  This files essentially processes the command-line arguments and decides
  what sounds to play. The real work is done in tonegen.c

==========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <wchar.h>
#include <time.h>
#include <alsa/asoundlib.h>
#include "tonegen.h" 

int main(void) {
  int volume = 100;

  snd_pcm_t *handle;
  snd_pcm_sframes_t period_size;
  const char *device = "hw:1,0"; 
  //if (!device) device = "default";
  if (tonegen_setup_sound (&handle, device, &period_size))
    {
    printf("period_size=%ld\n", period_size);
    tonegen_play_sound (handle, sound_type_tone, 0, volume,
       1000, 0, 600, 0, period_size); 
    tonegen_wait (handle);
    }
  return 0;
}
