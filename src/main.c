#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//TODO PUT INTO FILES

///////////////////
//  SDR_TERM     //
///////////////////

//Description//
//-----------//
//An SDR is an blocksay of a specific number of 128 bit blocks
//(that way no Hash ops are necessary, it's faster for this SDR size)


//Parameters//
//----------//
#define SDR_TERM_SIZE 2048
#define SDR_TERM_ONES 5
#define TERMS_MAX 100
#define SDR_BLOCK_TYPE __uint128_t
#define SDR_BLOCK_SIZE sizeof(SDR_BLOCK_TYPE)

//Data structure//
//--------------//
#define SDR_NUM_BLOCKS SDR_TERM_SIZE / SDR_BLOCK_SIZE
typedef struct
{
	SDR_BLOCK_TYPE blocks[SDR_NUM_BLOCKS];
}SDR;
SDR input_terms[TERMS_MAX];
bool input_terms_used[TERMS_MAX];

//Macros//
//-------//
//Iterate all the blocks of an SDR
#define ITERATE_SDR_BLOCKS(I,CODE) {\
	for(int I=0; I<SDR_NUM_BLOCKS; I++)\
	{\
		CODE\
	}}
//Iterate all the bits of an SDR
#define ITERATE_SDR_BITS(I,J,CODE) {\
	for(int I=0; I<SDR_NUM_BLOCKS; I++)\
	{\
		for(int J=0; J<SDR_BLOCK_SIZE; J++)\
		{\
			CODE\
		}\
	}}
//Transformation for bit index abstraction across the array of blocks
#define SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i) \
	int block_i = bit_i / SDR_BLOCK_SIZE;\
	int block_bit_i = bit_i % SDR_BLOCK_SIZE;\

//Methods//
//-------//
//Get an SDR for the input term which is a number from 1 to TERMS_MAX
SDR* getTerm(int number)
{
	if(input_terms_used[number])
		return &(input_terms[number]);
	for(int i=0; i<SDR_TERM_ONES; i++)
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
//Read the jth bit in the ith block of the SDR
int SDRReadBitInBlock(SDR *sdr, int block_i, int block_bit_j)
{
	return (sdr->blocks[block_i] >> block_bit_j) & 1;
}
//Write the jth bit in the ith block of the SDR with value
void SDRWriteBitInBlock(SDR *sdr, int block_i, int block_bit_j, int value)
{
	sdr->blocks[block_i] = (sdr->blocks[block_i] & (~(1 << block_bit_j))) | (value << block_bit_j);
}
//Read the ith bit in the SDR:
int SDRReadBit(SDR *sdr, int bit_i)
{
	SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i)
	return SDRReadBitInBlock(sdr, block_i, block_bit_i);
}
//Write the ith bit in the SDR with value:
void SDRWriteBit(SDR *sdr, int bit_i, int value)
{
	SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i)
	SDRWriteBitInBlock(sdr, block_i, block_bit_i, value);
}
//Print a SDR including zero bits
void printSDRFull(SDR *sdr)
{
	ITERATE_SDR_BITS(i,j,
		printf("%d,",(int) SDRReadBitInBlock(sdr,i,j));
	)
	printf("\n");
}

// print indices of true bits
void printSDRWhereTrue(SDR *sdr) {
	ITERATE_SDR_BITS(i,j,
		if (SDRReadBitInBlock(sdr,i,j)) {
			printf("(%d,%d)\n", i, j);
		}
	)
	printf("===\n");
}

//One SDR minus the other
SDR SDRMinus(SDR a, SDR b)
{
	SDR c;
	ITERATE_SDR_BLOCKS(i,
		c.blocks[i] = a.blocks[i] & (~b.blocks[i]);
	)
	return c;
}
//Union of both SDR's
SDR SDRUnion(SDR a, SDR b)
{
	SDR c;
	ITERATE_SDR_BLOCKS(i,
		c.blocks[i] = a.blocks[i] | b.blocks[i];
	)
	return c;
}
//permutation for sequence encoding
int seq_permutation[SDR_TERM_SIZE];
int seq_permutation_inverse[SDR_TERM_SIZE];
void initSequPermutation()
{
	for(int i=0; i<=SDR_TERM_SIZE-2; i++)
	{
		//choose an random integer so that 0<=i<=j<=SDR_TERM_SIZE
		int j = i+(random() % (SDR_TERM_SIZE-i));
		seq_permutation[i] = j;
		seq_permutation_inverse[j] = i;
	}
}
//Swap two bits in the SDR
void swap(SDR *sdr, int bit_i, int bit_j)
{
	//temp <- a, then a <- b, then b <- temp
	int temp = SDRReadBit(sdr, bit_i);
	SDRWriteBit(sdr, bit_i, SDRReadBit(sdr, bit_j));
	SDRWriteBit(sdr, bit_j, temp);
}
//Copy SDR:
SDR SDRCopy(SDR original)
{
	SDR c;
	ITERATE_SDR_BLOCKS(i,
		c.blocks[i] = original.blocks[i];
	)
	return c;
}
//Apply the seq_permutation to the SDR
SDR applySeqPermutation(SDR sdr, bool forward)
{
	SDR c = SDRCopy(sdr);
	for(int i=0; i<SDR_TERM_SIZE; i++)
	{
		swap(&c, i, forward ? seq_permutation[i] : seq_permutation_inverse[i]);
	}
	return c;
}
//Set can be made by simply using the SDRUnion
SDR SDRSet(SDR a, SDR b) {
	return SDRUnion(a, b);
}

//Tuple on the other hand:
SDR SDRTuple(SDR *a, SDR *b)
{
	SDR bPerm = applySeqPermutation(*b,true);
	return SDRSet(*a, bPerm);	
}


//Module Init//
//-----------//
void SDR_INIT()
{
	initSequPermutation();
}

///////////////////////
//  END SDR_TERM     //
///////////////////////
int main() 
{
	SDR_INIT();
	SDR *mySDR = getTerm(1);
	printSDRWhereTrue(mySDR);
	//not ready yet:
	SDR sdr2 = applySeqPermutation(*mySDR, true);
	printSDRWhereTrue(&sdr2);

	return 0;
}
