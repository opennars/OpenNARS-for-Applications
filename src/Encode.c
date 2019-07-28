#include "Encode.h"

//inspired by https://arxiv.org/pdf/1602.05925.pdf
//but the bucket always being half of the SDR size +min_overlap
//as needed for continuous perception purposes
SDR Encode_Scalar(int min, int max, int value)
{
    //min_overlap>0 guarantees continuous perception!
    int min_overlap = TERM_ONES; //Allow at least an overlap of the amount of 1-bits used for term encoding
    int range = max - min;
    int relative = value - min;
    int bucketsize = (SDR_SIZE/2 + min_overlap);
    int available_places = MAX(0, SDR_SIZE - bucketsize);
    double reached = ((double)relative) / ((double)range); //in [0,1]
    int max_index = (int) (reached * ((double) available_places));
    SDR result = {0};
    for(int i=max_index; i<max_index+bucketsize; i++)
    {
        SDR_WriteBit(&result, i, 1);
    }
    return result;
}

//A fast hash function for strings
//https://stackoverflow.com/questions/7666509/hash-function-for-string
//answer by cnicutar:
static unsigned long hash(unsigned char *str)
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
