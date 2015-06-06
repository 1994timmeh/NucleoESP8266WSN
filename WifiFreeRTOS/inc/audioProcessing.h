#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include "stm32f4xx_hal_conf.h"
#include "arm_math.h"

#define AUDIO_FRAME_LENGTH	256
#define FFT_LENGTH	512

#define SLIDING_WINDOW_LENGTH	3
#define SLIDING_WINDOW_SHIFT	3

#define NUM_FREQUENCIES			5

struct frameResults {
	uint8_t validFrame;
	int32_t maxBin[NUM_FREQUENCIES];
	float32_t maxValue;
	float32_t power;
	float32_t mean;
	float32_t variance;
	float32_t stdDev;
	float32_t skew;
	float32_t kurtosis;
};

void audioProcessingInit(void);
void audioProcessFrame(float32_t* micOneData, float32_t* micTwoData, struct frameResults* results);
void serialize_results(struct frameResults results, uint8_t* x);

#endif
