#include "Encode.h"

SDR Encode_Scalar(int w, int min, int max, int value)
{
    int n = SDR_SIZE;
    int numberOfBuckets = n - w + 1;
    int range = max - min;
    int relative = value - min;
    // determine bucket into which the number falls into
    // see https://arxiv.org/pdf/1602.05925.pdf
    int selectedBucket = (int)floor((double)numberOfBuckets * relative / range);
    SDR result = {0};
    // active bits as described in the paper
    for (int bitIdx=selectedBucket; bitIdx<selectedBucket+w; bitIdx++)
    {
        SDR_WriteBit(&result, bitIdx, 1);
    }
    return result;
}

//A fast hash function for strings
//https://stackoverflow.com/questions/7666509/hash-function-for-string
//answer by cnicutar:
unsigned long hash(unsigned char *str)
{
    unsigned long hash = 5381;
    int c;
    while ((c = *str++))
    {
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
    }
    return hash;
}

SDR Encode_Term(char *name)
{
    int number = hash((unsigned char*)name)%TERMS_MAX;
    if(input_terms_used[number])
    {
        return input_terms[number];
    }
    for(int i=0; i<TERM_ONES; i++)
    {
        //1. choose which block to go into
        int block_i = rand() % SDR_NUM_BLOCKS;
        //2. choose at which position to put 1
        int bit_j = rand() % SDR_BLOCK_SIZE;
        //3. put it there
        SDR_WriteBitInBlock(&input_terms[number], block_i, bit_j, 1);
    }
    input_terms_used[number] = true;
    return input_terms[number];
}
