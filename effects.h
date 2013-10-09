#include "equaliser.h"

void ringModulation (int16_t * playbackBuffer, float ringFreq);
void echo (int16_t * playbackBuffer, uint16_t samples);
void vibrato (int16_t * playbackBuffer, uint16_t samples);
void lpf (int16_t * playbackBuffer);
void reverse (int16_t * playbackBuffer);
void equaliser3 (int16_t * playbackBuffer, EQSTATE eq);
void reverb (int16_t * playbackBuffer, uint16_t N);
void hpf (int16_t * playbackBuffer, uint16_t cutoff);
