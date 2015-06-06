

#include "audioProcessing.h"
#include "audioStatistics.h"
#include "arm_const_structs.h"

#include "board.h"
#include "stm32f4xx_hal_conf.h"

#define AUDIODEBUG
#define DEBUG_PINS

void arm_copy_complex(float32_t* pSrc, float32_t* pDst, uint32_t blockSize);

// FFT Arrays
float32_t micOneFFTdata[FFT_LENGTH];
float32_t micTwoFFTdata[FFT_LENGTH];
float32_t micOneFFT[2*FFT_LENGTH];
float32_t micTwoFFT[2*FFT_LENGTH];
float32_t combinedData[2*FFT_LENGTH];

// FFT Variables
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
arm_rfft_fast_instance_f32 fftStructures;

int32_t pastBins[SLIDING_WINDOW_LENGTH];

void audioProcessingInit(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	uint8_t i;
	arm_rfft_fast_init_f32(&fftStructures, FFT_LENGTH);
	for (i = 0; i < SLIDING_WINDOW_LENGTH; i++) {
		pastBins[i] = 2*AUDIO_FRAME_LENGTH;
	}


	#ifdef DEBUG_PINS

	/* Enable the D8 & D9 Clock */
  	__BRD_D8_GPIO_CLK();
	__BRD_D9_GPIO_CLK();

  	/* Configure the D8 pin as an output */
	GPIO_InitStructure.Pin = BRD_D8_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D8_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

  	/* Configure the D9 pin as an output */
	GPIO_InitStructure.Pin = BRD_D9_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_D9_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	HAL_GPIO_WritePin(BRD_D8_GPIO_PORT, BRD_D8_PIN, 0x00);
	HAL_GPIO_WritePin(BRD_D9_GPIO_PORT, BRD_D9_PIN, 0x00);

	#endif /* DEBUG_PINS */

}

void audioProcessFrame(float32_t* micOneData, float32_t* micTwoData, struct frameResults* results) {
	uint8_t i, consecutiveFrame;
	float32_t maxValue;
	uint32_t maxBin;
	float32_t temp;

#ifdef DEBUG_PINS
	HAL_GPIO_WritePin(BRD_D8_GPIO_PORT, BRD_D8_PIN, 0x01);
#endif /* DEBUG PINS */

	// Initialise both data sequences as 0.0
	arm_fill_f32(0.0,  micOneFFTdata, FFT_LENGTH);
	arm_fill_f32(0.0,  micTwoFFTdata, FFT_LENGTH);

	// Fill both data arrays with audio data, mic two data is reversed
	arm_copy_f32(micOneData,  micOneFFTdata, AUDIO_FRAME_LENGTH);
	arm_copy_f32(micTwoData + AUDIO_FRAME_LENGTH,  micTwoFFTdata, AUDIO_FRAME_LENGTH);

	// Perform complex fft on mic data
	ifftFlag = 0;
	arm_rfft_fast_f32(&fftStructures, micOneFFTdata, micOneFFT, ifftFlag);
	arm_rfft_fast_f32(&fftStructures, micTwoFFTdata, micTwoFFT, ifftFlag);

	// Multiply
	arm_cmplx_mult_cmplx_f32(micOneFFT, micTwoFFT, combinedData, FFT_LENGTH);
	// Because first complex byte contains two packed real values
	combinedData[0] = micOneFFT[0] * micTwoFFT[0];
	combinedData[1] = micOneFFT[1] * micTwoFFT[1];

	// Perform IFFT
	ifftFlag = 1;
	arm_rfft_fast_f32(&fftStructures, combinedData, micOneFFTdata, ifftFlag);

	// Find maximum  correlation points
	arm_max_f32(micOneFFTdata, FFT_LENGTH, &maxValue, &(maxBin));
	results->maxBin[0] = maxBin;
	for (i = 0; i < NUM_FREQUENCIES; i++) {
		arm_max_f32(micOneFFTdata, FFT_LENGTH, &temp, &(results->maxBin[i]));
		results->maxBin[i] = (AUDIO_FRAME_LENGTH - 1) - results->maxBin[i];
	}

	// Compare maximum bin to previous bins
	consecutiveFrame = 1;
	for (i = 0; i < SLIDING_WINDOW_LENGTH; i++) {
		// If its out of range, no inherent structure to signal
		if (maxBin > pastBins[i] + SLIDING_WINDOW_SHIFT) {
			consecutiveFrame = 0;
		} else if (maxBin < pastBins[i] - SLIDING_WINDOW_SHIFT) {
			consecutiveFrame = 0;
		}
	}
	// Update list
	for (i = 0; i < SLIDING_WINDOW_LENGTH - 1; i++) {
		pastBins[i+1] = pastBins[i];
	}
	pastBins[0] = maxBin;

#ifdef DEBUG_PINS
	HAL_GPIO_WritePin(BRD_D8_GPIO_PORT, BRD_D8_PIN, 0x00);
	HAL_GPIO_WritePin(BRD_D9_GPIO_PORT, BRD_D9_PIN, 0x01);
#endif /* DEBUG PINS */

	arm_cmplx_mag_f32(combinedData, micOneFFTdata, FFT_LENGTH);	

	audioStatsPower(micOneFFTdata, FFT_LENGTH, &(results->power));
	audioStatsMean(micOneFFTdata, FFT_LENGTH, &(results->mean));
	audioStatsVariance(micOneFFTdata, FFT_LENGTH, &(results->variance));

	audioStatsStdDev(results->variance, &(results->stdDev));
	audioStatsSkewness(micOneFFTdata, results->mean, results->stdDev, FFT_LENGTH, &(results->skew));
	audioStatsKurtosis(micOneFFTdata, results->mean, results->stdDev, FFT_LENGTH, &(results->kurtosis));

#ifdef DEBUG_PINS
	HAL_GPIO_WritePin(BRD_D9_GPIO_PORT, BRD_D9_PIN, 0x00);
#endif /* DEBUG PINS */

	// Populate remaining result structure
	results->validFrame = consecutiveFrame;
	results->maxValue = maxValue;

#ifdef AUDIODEBUG
	if(consecutiveFrame){
		debug_printf("maxValue: %d ", (int) (maxValue));
		debug_printf("maxBin: %d\n", (int) maxBin);
	}

#endif

	return;
}

void arm_copy_complex(float32_t* pSrc, float32_t* pDst, uint32_t blockSize) {
	uint32_t blkCnt;
	float32_t in1, in2, in3, in4;
	blkCnt = blockSize >> 2u;
	while(blkCnt > 0u) {
		/* C = A */
		/* Copy and then store the results in the destination buffer */
		in1 = *pSrc++;
		in2 = *pSrc++;
		in3 = *pSrc++;
		in4 = *pSrc++;

		/* Shifting into complex array, src only contains real data*/
		*(pDst++) = in1;
		pDst++;
		*(pDst++) = in2;
		pDst++;
		*(pDst++) = in3;
		pDst++;
		*(pDst++) = in4;
		pDst++;
		/* Decrement the loop counter */
		blkCnt--;
	}
}

/**
 * Prints a human readable version of the results struct
 * @param results - Pointer to a frameResults struct
 * @throws none
 */
void print_results(struct frameResults results){
	debug_printf("=====RESULTS=====\n");
	debug_printf("validFrame: %d\n", results.validFrame);
	debug_printf("maxValue: %f\n", results.maxValue);
	debug_printf("maxBin: %d\n\n", results.maxBin);
}

//struct frameResults {
//	uint8_t validFrame;
//	int32_t maxBin[NUM_FREQUENCIES];
//	float32_t maxValue;
//	float32_t power;
//	float32_t mean;
//	float32_t variance;
//	float32_t stdDev;
//	float32_t skew;
//	float32_t kurtosis;
//};
void serialize_results(struct frameResults results, uint8_t* x){
	x[0] = (uint8_t)results.validFrame;

	uint32_t maxBin = (uint32_t)results.maxBin[0];
	x[1] = (maxBin & 0x000000FF);
	x[2] = (maxBin & 0x0000FF00) >> 8;
	x[3] = (maxBin & 0x00FF0000) >> 16;
	x[4] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.maxBin[1];
	x[5] = (maxBin & 0x000000FF);
	x[6] = (maxBin & 0x0000FF00) >> 8;
	x[7] = (maxBin & 0x00FF0000) >> 16;
	x[8] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.maxBin[2];
	x[9] = (maxBin & 0x000000FF);
	x[10] = (maxBin & 0x0000FF00) >> 8;
	x[11] = (maxBin & 0x00FF0000) >> 16;
	x[12] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.maxBin[3];
	x[13] = (maxBin & 0x000000FF);
	x[14] = (maxBin & 0x0000FF00) >> 8;
	x[15] = (maxBin & 0x00FF0000) >> 16;
	x[16] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.maxBin[4];
	x[17] = (maxBin & 0x000000FF);
	x[18] = (maxBin & 0x0000FF00) >> 8;
	x[19] = (maxBin & 0x00FF0000) >> 16;
	x[20] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.maxValue;
	x[21] = (maxBin & 0x000000FF);
	x[22] = (maxBin & 0x0000FF00) >> 8;
	x[23] = (maxBin & 0x00FF0000) >> 16;
	x[24] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.power;
	x[25] = (maxBin & 0x000000FF);
	x[26] = (maxBin & 0x0000FF00) >> 8;
	x[27] = (maxBin & 0x00FF0000) >> 16;
	x[28] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.mean;
	x[29] = (maxBin & 0x000000FF);
	x[30] = (maxBin & 0x0000FF00) >> 8;
	x[31] = (maxBin & 0x00FF0000) >> 16;
	x[32] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.variance;
	x[33] = (maxBin & 0x000000FF);
	x[34] = (maxBin & 0x0000FF00) >> 8;
	x[35] = (maxBin & 0x00FF0000) >> 16;
	x[36] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.stdDev;
	x[37] = (maxBin & 0x000000FF);
	x[38] = (maxBin & 0x0000FF00) >> 8;
	x[39] = (maxBin & 0x00FF0000) >> 16;
	x[40] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.skew;
	x[41] = (maxBin & 0x000000FF);
	x[42] = (maxBin & 0x0000FF00) >> 8;
	x[43] = (maxBin & 0x00FF0000) >> 16;
	x[44] = (maxBin & 0xFF000000) >> 24;

	maxBin = (uint32_t)results.kurtosis;
	x[45] = (maxBin & 0x000000FF);
	x[46] = (maxBin & 0x0000FF00) >> 8;
	x[47] = (maxBin & 0x00FF0000) >> 16;
	x[48] = (maxBin & 0xFF000000) >> 24;

	x[49] = 0x00;

//	int i = 0;
//	for(;i<12;i++){
//		debug_printf("0x%02x ", x[i]);
//	}
}

