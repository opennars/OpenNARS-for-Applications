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
//Init module
void SDR_INIT();
//Get an SDR for the input term which is a number from 1 to TERMS_MAX
SDR* getTerm(int number);
//Read the jth bit in the ith block of the SDR
int SDRReadBitInBlock(SDR *sdr, int block_i, int block_bit_j)
//Write the jth bit in the ith block of the SDR with value
void SDRWriteBitInBlock(SDR *sdr, int block_i, int block_bit_j, int value)
//Read the ith bit in the SDR:
int SDRReadBit(SDR *sdr, int bit_i)

//Write the ith bit in the SDR with value:
void SDRWriteBit(SDR *sdr, int bit_i, int value)
//Print a SDR including zero bits
void printSDRFull(SDR *sdr)
// print indices of true bits
void printSDRWhereTrue(SDR *sdr)
//One SDR minus the other
SDR SDRMinus(SDR a, SDR b)
//Union of both SDR's
SDR SDRUnion(SDR a, SDR b)
//Create a copy of the SDR
SDR SDRCopy(SDR original);
//Apply the seq_permutation to the SDR
SDR Permute(SDR sdr, bool forward);
//Set can be made by simply using the SDRUnion
SDR SDRSet(SDR a, SDR b);
//Tuple on the other hand:
SDR SDRTuple(SDR *a, SDR *b);
//Sequence
SDR SDRSequence(SDR a, SDR b);
//Implication
SDR SDRImplication(SDR a, SDR b);
//Inheritance
SDR SDRInheritance(SDR a, SDR b);
//Match confidence when matching the part SDR to the full
double SDRMatch(SDR part,SDR full);
//Whether a SDR is of a certain type (type is also encoded in the SDR)
double SDRTermType(SDR type, SDR sdr);
//Equality is symmetric:
double SDREqualTerm(SDR a, SDR b);

#endif
