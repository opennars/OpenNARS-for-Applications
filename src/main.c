//TODO PUT INTO FILES

///////////////////
//  SDR_TERM     //
///////////////////

//Description
//An SDR is an blocksay of a specific number of 128 bit blocks
//(that way no Hash ops are necessary, it's faster for this SDR size)

//Parameters
#define SDR_TERM_SIZE 2048
#define SDR_TERM_ONES 5
#define TERMS_MAX 100
#define SDR_BLOCK_TYPE __uint128_t
#define SDR_BLOCK_SIZE sizeof(SDR_BLOCK_TYPE)

//Data structure
#define SDR_NUM_BLOCKS SDR_TERM_SIZE / SDR_BLOCK_SIZE
typedef struct
{
	SDR_BLOCK_TYPE blocks[SDR_NUM_BLOCKS];
}SDR;
SDR input_terms[TERMS_MAX];
_Bool input_terms_used[TERMS_MAX];

//-------//
//Methods//
//-------//

//Get an SDR for the input term which is a number from 1 to TERMS_MAX
SDR getTerm(int number)
{
	if(input_terms_used[number])
		return input_terms[number];
	for(int i=0; i<SDR_TERM_ONES; i++)
	{
		//1. choose which block to go into
		int block_i = rand() % SDR_NUM_BLOCKS;
		//2. choose at which position to put 1
		int bit_j = rand() % SDR_BLOCK_SIZE;
		//3. put it there
		input_terms[number].blocks[block_i] |= (1 << bit_j);
	}
	return input_terms[number];
}

void printSDRFull(SDR sdr)
{
	for(int i=0; i<SDR_NUM_BLOCKS; i++)
	{
		for(int j=0; j<SDR_BLOCK_SIZE; j++)
		{
			printf("%d,",(int)(sdr.blocks[i] >> j) & 1);
		}
	}
}

///////////////////////
//  END SDR_TERM     //
///////////////////////

void main() 
{
	SDR mySDR = getTerm(1);
	printSDRFull(mySDR);
} 
