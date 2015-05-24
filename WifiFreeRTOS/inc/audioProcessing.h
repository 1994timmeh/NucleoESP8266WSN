#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include "stm32f4xx_hal_conf.h"
#include "arm_math.h"

#define AUDIO_FRAME_LENGTH	256
#define FFT_LENGTH	512

#define SLIDING_WINDOW_LENGTH	3
#define SLIDING_WINDOW_SHIFT	3


struct frameResults {
	uint8_t validFrame;
	float maxValue;
	int32_t maxBin;
};

void audioProcessingInit(void);
void audioProcessFrame(float32_t* micOneData, float32_t* micTwoData, struct frameResults* results);

#endif
