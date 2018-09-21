#ifndef SDR_H
#define SDR_H

///////////////////
//  SDR_TERM     //
///////////////////

//Description//
//-----------//
//An SDR is an blocksay of a specific number of 128 bit blocks
//(that way no Hash ops are necessary, it's faster for this SDR size)

//References//
//-----------//
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//Parameters//
//----------//
#define SDR_SIZE 2048
#define SDR_ONES 5
#define SDR_BLOCK_TYPE __uint128_t
#define SDR_BLOCK_SIZE sizeof(SDR_BLOCK_TYPE)

//Data structure//
//--------------//
#define SDR_NUM_BLOCKS SDR_SIZE / SDR_BLOCK_SIZE
typedef struct
{
    SDR_BLOCK_TYPE blocks[SDR_NUM_BLOCKS];
}SDR;

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
//Init module
void SDR_INIT();
//Get an SDR for the input term which is a number from 1 to TERMS_MAX
SDR* SDR_EncodeTerm(int number);
//Read the jth bit in the ith block of the SDR
int SDR_ReadBitInBlock(SDR *sdr, int block_i, int block_bit_j);
//Write the jth bit in the ith block of the SDR with value
void SDR_WriteBitInBlock(SDR *sdr, int block_i, int block_bit_j, int value);
//Read the ith bit in the SDR:
int SDR_ReadBit(SDR *sdr, int bit_i);
//Write the ith bit in the SDR with value:
void SDR_WriteBit(SDR *sdr, int bit_i, int value);
//Print a SDR including zero bits
void SDR_PrintFull(SDR *sdr);
// print indices of true bits
void SDR_PrintWhereTrue(SDR *sdr);
//One SDR minus the other
SDR SDR_Minus(SDR a, SDR b);
//Union of both SDR's
SDR SDR_Union(SDR a, SDR b);
//Intersection of both SDR's
SDR SDR_Intersection(SDR a, SDR b);
//Xor of both SDR's
SDR SDR_Xor(SDR a, SDR b);
//Create a copy of the SDR
SDR SDR_Copy(SDR original);
//Apply the seq_permutation to the SDR
SDR SDR_Permute(SDR sdr, bool forward);
//Set can be made by simply using the SDRUnion
SDR SDR_Set(SDR a, SDR b);
//Tuple on the other hand:
SDR SDR_Tuple(SDR *a, SDR *b);
//Get the other element of the tuple
SDR SDR_TupleGetFirstElement(SDR *compound, SDR *secondElement);
SDR SDR_TupleGetSecondElement(SDR *compound, SDR *firstElement);
//Match confidence when matching the part SDR to the full
double SDR_Match(SDR *part, SDR *full);
//The confidence measuring to what degree a SDR is a special case of another
double SDR_Inheritance(SDR full, SDR part);
//The confidence measuring to what degree a SDR is similar to another
double SDR_Similarity(SDR a, SDR b);
//Equality is symmetric:
double SDR_EqualTerm(SDR *a, SDR *b);

#endif
