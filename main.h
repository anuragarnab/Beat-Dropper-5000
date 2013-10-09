#define SOUNDSIZE (6044) // size of sample16bits ie snare
#define SOUNDSIZE2 (12701) // size of fxTom
#define SOUNDSIZE3 (12690) // size of kick
#define SOUNDSIZE4 (14736) // size of hihat
#define BUFFERSIZE (32768)

// Effect id's
static const uint8_t NO_EFFECT = 99;
static const uint8_t ECHO = 0;
static const uint8_t LPF = 1;
static const uint8_t HPF = 2;
static const uint8_t RING = 3;
static const uint8_t EQ3 = 4;
static const uint8_t VIBRATO = 5;
static const uint8_t REVERB = 6;

#include "lpf.h"
