#include "stm32f4xx.h"
#include "main.h"
#include "effects.h"

void reverse (int16_t * playbackBuffer){
	uint16_t i = 0;
	uint16_t temp = 0;

	for (; i < (BUFFERSIZE>>1); ++i){
		temp = playbackBuffer[i];
		playbackBuffer[i] = playbackBuffer[BUFFERSIZE-i-1];
		playbackBuffer[BUFFERSIZE-i-1] = temp;
	}
}

void echo (int16_t * playbackBuffer, uint16_t N){

	uint16_t i = 0;

	if (N > BUFFERSIZE){
		return;
	}

	i = N;

	for (; i < BUFFERSIZE; ++i){
		playbackBuffer [i] = playbackBuffer[i] + 0.5*playbackBuffer [i - N];
	}
}

void reverb (int16_t * playbackBuffer, uint16_t N){

	uint16_t i = 0;

	if (N > BUFFERSIZE){
		return;
	}

	i = N;
	uint8_t j = 1;
	uint8_t divisor = 2;

	for (; i < BUFFERSIZE; ++i){
		if (i % N == 0 && i != 0){
			++divisor;
		}
		playbackBuffer [i] = playbackBuffer[i]/divisor;

		for (; j < divisor; ++j){
			playbackBuffer[i] += (playbackBuffer[i - N]/(divisor<<j));
		}

		j = 1;
	}
}

void ringModulation (int16_t * playbackBuffer, float frequency){

	uint16_t i = 0;
	float ftemp = 0.0f;
	float j = 0;

	for (; i < BUFFERSIZE; ++i, ++j){
		ftemp = sinf (j / frequency);
		playbackBuffer[i] =  playbackBuffer[i] * ftemp;
	}
}

void lpf (int16_t * playbackBuffer){
	uint16_t i = 0;
	for ( ; i < BUFFERSIZE; ++i){
		playbackBuffer[i] = (uint16_t) (( LPfilter_processSample( playbackBuffer[i] / 32767.0f) ) * 32767.0f );
	}
}

void equaliser3 (int16_t * playbackBuffer, EQSTATE eq){
	uint16_t i = 0;
	for ( ; i < BUFFERSIZE; ++i){
		playbackBuffer[i] = (uint16_t) (   do_3band(&eq, (float) playbackBuffer[i])  );
	}
}

/*
 * Haven't checked for clipping
 * Delayed sample should be N/22050 seconds
 */
void vibrato (int16_t * playbackBuffer, uint16_t N){

	uint16_t i = 0;

	if (N > BUFFERSIZE){
		return;
	}

	i = N;
	float temp;
	float j = i;
	float w = 2.0 * (22.0/7.0) * 30; // w = 2*pi * 30 Hz

	for (; i < BUFFERSIZE-N; ++i, ++j){
		temp = N * sinf (w * (j / 32768.0f));
		playbackBuffer [i] = playbackBuffer[i] + 0.5*playbackBuffer [i - (uint16_t) temp];
	}
}

void hpf (int16_t * playbackBuffer, uint16_t cutoff){
    float RC = 1.0/(cutoff*2*3.14);
    float dt = 1.0/22050; // sample rate is 22050
    float alpha = RC/(RC + dt);
    uint16_t i = 1;

    int16_t x1 = playbackBuffer[1];
    int16_t x0 = playbackBuffer[0];

    for (; i<BUFFERSIZE; ++i){
    	playbackBuffer[i] = alpha * (playbackBuffer[i-1] + x1 - x0);
    	x0 = x1;
    	x1 = playbackBuffer[i+1];

    }
}
