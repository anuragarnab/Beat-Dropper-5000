/*
 * Hopefully a low pass filter
 */

#define _2PI                    6.283185307f
#define _PI                    	3.14159265f
#define SAMPLERATE				44100

#ifndef _LPFILTER_H
#define _LPFILTER_H

#include <math.h>
//----------------------------------------------------------------------
void 	LPfilter_reset(void);
void 	LPfilter_computeCoeff (float fc, float res);
float	LPfilter_processSample	(float input);
//----------------------------------------------------------------------

#endif
