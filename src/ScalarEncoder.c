#include <string.h>

#include "ScalarEncoder.h"


// https://www.youtube.com/watch?v=V3Yqtpytif0&list=PL3yXMgtrZmDqhsFQzwUC9V8MeeVOQ7eZ9&index=6
SDR encoder_scalar(int w, int min, int max, int value) {
	int n = SDR_TERM_SIZE;

	int numberOfBuckets = n - w - 1;

	int range = max - min;
	int relative = value - min;

	// determine bucket into which the number falls into
	// see https://arxiv.org/pdf/1602.05925.pdf
	int selectedBucket = (int)floor((double)numberOfBuckets * relative / range);

	SDR result;
	memset(&result, 0, sizeof(SDR));

	// active bits as described in the paper
	for (int bitIdx=selectedBucket; bitIdx<selectedBucket+w; bitIdx++) {
		SDRWriteBit(&result, bitIdx, 1);
	}

	return result;
}
