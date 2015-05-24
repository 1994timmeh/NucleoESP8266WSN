#include "audioStatistics.h"


void audioStatsStdDev(float32_t variance, float32_t *stdDev) {
	arm_sqrt_f32(variance, stdDev);
}

void audioStatsSkewness(float32_t *pSrc, float32_t mean, float32_t stdDev, uint32_t blockSize, float32_t *pResults) {
	uint32_t blkCnt, i;
	float32_t x;
	*pResults = 0.0;

	blkCnt = blockSize >> 2u;
	
	for (i = 0; i < blkCnt; i++) {
		x = ((*pSrc++) - mean) / stdDev;
		*pResults += ( x * x * x );

		x = ((*pSrc++) - mean) / stdDev;
		*pResults += ( x * x * x );

		x = ((*pSrc++) - mean) / stdDev;
		*pResults += ( x * x * x );

		x = ((*pSrc++) - mean) / stdDev;
		*pResults += ( x * x * x );
	}
	*pResults /= blockSize;
}

void audioStatsKurtosis(float32_t *pSrc, float32_t mean, float32_t stdDev, uint32_t blockSize, float32_t *pResults) {
	uint32_t blkCnt, i;
	float32_t x, x_i = 0.0;
	*pResults = 0.0;

	blkCnt = blockSize >> 2u;
	
	for (i = 0; i < blkCnt; i++) {
		x_i = *pSrc++;
		if (x_i > 0.0) {
			x = (x_i - mean) / stdDev;
			*pResults += ( x * x * x * x );
		}
		x_i = *pSrc++;
		if (x_i > 0.0) {
			x = (x_i - mean) / stdDev;
			*pResults += ( x * x * x * x );
		}
		x_i = *pSrc++;
		if (x_i > 0.0) {
			x = (x_i - mean) / stdDev;
			*pResults += ( x * x * x * x );
		}
		x_i = *pSrc++;
		if (x_i > 0.0) {
			x = (x_i - mean) / stdDev;
			*pResults += ( x * x * x * x );
		}
	}
	*pResults /= blockSize;
	*pResults -= 3;
}
