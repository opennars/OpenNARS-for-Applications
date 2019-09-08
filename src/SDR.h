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
#include <stdint.h>
#include "Truth.h"

//Parameters//
//----------//
#define SDR_SIZE 2048
#ifndef SDR_BLOCK_TYPE
#define SDR_BLOCK_TYPE uintmax_t
#endif
#define SDR_BLOCK_SIZE (8*sizeof(SDR_BLOCK_TYPE))
#define SDR_HASH_TYPE uint64_t
#define SDR_HASH_TYPE_SIZE (8*sizeof(SDR_HASH_TYPE))

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
    for(unsigned int I=0; I<SDR_NUM_BLOCKS; I++)\
    {\
        CODE\
    }}
//Iterate all the bits of an SDR
#define ITERATE_SDR_BITS(I,J,CODE) {\
    for(unsigned int I=0; I<SDR_NUM_BLOCKS; I++)\
    {\
        for(unsigned int J=0; J<SDR_BLOCK_SIZE; J++)\
        {\
            CODE\
        }\
    }}
//Transformation for bit index abstraction across the array of blocks
#define SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i) \
    int block_i = bit_i / SDR_BLOCK_SIZE;\
    int block_bit_i = bit_i % SDR_BLOCK_SIZE;

//Methods//
//-------//
//Read the jth bit in the ith block of the SDR
int SDR_ReadBitInBlock(SDR *sdr, int block_i, int block_bit_j);
//Write the jth bit in the ith block of the SDR with value
void SDR_WriteBitInBlock(SDR *sdr, int block_i, int block_bit_j, int value);
//Write the ith bit in the SDR with value:
void SDR_WriteBit(SDR *sdr, int bit_i, int value);
// count amount of true bits
int SDR_CountTrue(SDR *sdr);
// print indices of true bits
void SDR_Print(SDR *sdr);
//Generate a random permutation and its inverse
void SDR_GeneratePermutation(int *perm, int *perm_inverse);
//Apply the seq_permutation to the SDR
SDR SDR_Permute(SDR *sdr, int *permutation);
//Two faster permutations
SDR SDR_PermuteByRotation(SDR *sdr, bool forward);
//Tuple on the other hand:
SDR SDR_Tuple(SDR *a, SDR *b);
//Get the other element of the tuple
SDR SDR_TupleGetFirstElement(SDR *compound, SDR *secondElement);
SDR SDR_TupleGetSecondElement(SDR *compound, SDR *firstElement);
//Whether two SDR's are equal completely
bool SDR_Equal(SDR *a, SDR *b);
//The confidence measuring to what degree a SDR is a special case of another
//also see https://github.com/patham9/ANSNA/wiki/SDRInheritance-for-matching,-and-its-truth-value
Truth SDR_Inheritance(SDR *full, SDR *part);
//The confidence measuring to what degree a SDR is similar to another
Truth SDR_Similarity(SDR *a, SDR *b);
//Hash of SDR
SDR_HASH_TYPE SDR_Hash(SDR *sdr);
//SDR init
void SDR_INIT();

#endif
