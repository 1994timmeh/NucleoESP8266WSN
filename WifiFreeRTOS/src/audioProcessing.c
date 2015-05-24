

#include "audioProcessing.h"
#include "arm_const_structs.h"

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
	uint8_t i;
	arm_rfft_fast_init_f32(&fftStructures, FFT_LENGTH);
	for (i = 0; i < SLIDING_WINDOW_LENGTH; i++) {
		pastBins[i] = 2*AUDIO_FRAME_LENGTH;
	}
}

void audioProcessFrame(float32_t* micOneData, float32_t* micTwoData, struct frameResults* results) {
	uint8_t i, consecutiveFrame;
	float32_t maxValue;
	uint32_t maxBin;

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
	arm_max_f32(micOneFFTdata, FFT_LENGTH, &maxValue, &maxBin);
	maxBin = (AUDIO_FRAME_LENGTH - 1) - maxBin;

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

	// Populate result structure
	results->validFrame = consecutiveFrame;
	results->maxValue = maxValue;
	results->maxBin = maxBin;
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
