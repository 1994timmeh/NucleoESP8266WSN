#ifndef AUDIO_STATISTICS_H
#define AUDIO_STATISTICS_H

#include "stm32f4xx_hal_conf.h"
#include "arm_math.h"

#define audioStatsPower arm_power_f32
#define audioStatsMean arm_mean_f32
#define audioStatsVariance arm_var_f32

//void audioStatsPower(float32_t *pSrc, uint32_t blockSize, float32_t *pResults)
//void audioStatsMean(float32_t *pSrc, uint32_t blockSize, float32_t *pResults)
//void audioStatsVariance(float32_t *pSrc, uint32_t blockSize, float32_t *pResults)

void audioStatsStdDev(float32_t variance, float32_t *stdDev);
void audioStatsSkewness(float32_t *pSrc, float32_t mean, float32_t stdDev, uint32_t blockSize, float32_t *pResults);
void audioStatsKurtosis(float32_t *pSrc, float32_t mean, float32_t stdDev, uint32_t blockSize, float32_t *pResults);

#endif
