#include <string.h>
#include <math.h>
#include "Encode.h"

// https://www.youtube.com/watch?v=V3Yqtpytif0&list=PL3yXMgtrZmDqhsFQzwUC9V8MeeVOQ7eZ9&index=6
SDR Encode_Scalar(int w, int min, int max, int value) {
	int n = SDR_SIZE;
	int numberOfBuckets = n - w - 1;
	int range = max - min;
	int relative = value - min;
	// determine bucket into which the number falls into
	// see https://arxiv.org/pdf/1602.05925.pdf
	int selectedBucket = (int)floor((double)numberOfBuckets * relative / range);
	SDR result = {0};
	// active bits as described in the paper
	for (int bitIdx=selectedBucket; bitIdx<selectedBucket+w; bitIdx++) {
		SDR_WriteBit(&result, bitIdx, 1);
	}
	return result;
}

SDR* Encode_Term(int number)
{
    if(input_terms_used[number])
        return &(input_terms[number]);
    for(int i=0; i<SDR_ONES; i++)
    {
        //1. choose which block to go into
        int block_i = rand() % SDR_NUM_BLOCKS;
        //2. choose at which position to put 1
        int bit_j = rand() % SDR_BLOCK_SIZE;
        //3. put it there
        input_terms[number].blocks[block_i] |= (1 << bit_j);
    }
    input_terms_used[number] = true;
    return &(input_terms[number]);
}
