#include "SDR.h"

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

int SDRReadBitInBlock(SDR *sdr, int block_i, int block_bit_j)
{
    return (sdr->blocks[block_i] >> block_bit_j) & 1;
}

void SDRWriteBitInBlock(SDR *sdr, int block_i, int block_bit_j, int value)
{
    sdr->blocks[block_i] = (sdr->blocks[block_i] & (~(1 << block_bit_j))) | (value << block_bit_j);
}

int SDRReadBit(SDR *sdr, int bit_i)
{
    SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i)
    return SDRReadBitInBlock(sdr, block_i, block_bit_i);
}

void SDRWriteBit(SDR *sdr, int bit_i, int value)
{
    SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i)
    SDRWriteBitInBlock(sdr, block_i, block_bit_i, value);
}

void printSDRFull(SDR *sdr)
{
    ITERATE_SDR_BITS(i,j,
        printf("%d,",(int) SDRReadBitInBlock(sdr,i,j));
    )
    printf("\n");
}

void printSDRWhereTrue(SDR *sdr) {
    ITERATE_SDR_BITS(i,j,
        if (SDRReadBitInBlock(sdr,i,j)) {
            printf("(%d,%d)\n", i, j);
        }
    )
    printf("===\n");
}

SDR SDRMinus(SDR a, SDR b)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = a.blocks[i] & (~b.blocks[i]);
    )
    return c;
}

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

void swap(SDR *sdr, int bit_i, int bit_j)
{
    //temp <- a, then a <- b, then b <- temp
    int temp = SDRReadBit(sdr, bit_i);
    SDRWriteBit(sdr, bit_i, SDRReadBit(sdr, bit_j));
    SDRWriteBit(sdr, bit_j, temp);
}

SDR SDRCopy(SDR original)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = original.blocks[i];
    )
    return c;
}
SDR Permute(SDR sdr, bool forward)
{
    SDR c = SDRCopy(sdr);
    for(int i=0; i<SDR_TERM_SIZE; i++)
    {
        swap(&c, i, forward ? seq_permutation[i] : seq_permutation_inverse[i]);
    }
    return c;
}

SDR SDRSet(SDR a, SDR b)
{
    return SDRUnion(a, b);
}

SDR SDRTuple(SDR *a, SDR *b)
{
    SDR bPerm = Permute(*b,true);
    return SDRSet(*a, bPerm);    
}

SDR SDRSequence(SDR a, SDR b)
{
    return SDRSet(sequence, SDRTuple(&a,&b));
}

SDR implication;
SDR SDRImplication(SDR a, SDR b)
{
    return SDRSet(implication, SDRTuple(&a,&b));
}

SDR inheritance;
SDR SDRInheritance(SDR a, SDR b)
{
    return SDRSet(inheritance, SDRTuple(&a,&b));
}

double SDRMatch(SDR part,SDR full)
{
    int countOneInBoth = 0;
    int countOneInPart = 0;
    ITERATE_SDR_BITS(i,j,
        countOneInBoth += (SDRReadBitInBlock(&part,i,j) & SDRReadBitInBlock(&full,i,j));
        countOneInPart += SDRReadBitInBlock(&part,i,j);
    )
    return (double)countOneInBoth / countOneInPart;
}

double SDRTermType(SDR type, SDR sdr)
{
    return SDRMatch(type, sdr);
}

double SDREqualTerm(SDR a, SDR b)
{
    return SDRMatch(a, b) * SDRMatch(b, a);
}
//double SDRContainsSubterm( TODO

void SDR_INIT()
{
    initSequPermutation();
    sequence = *getTerm(TERMS_MAX-1);
    implication = *getTerm(TERMS_MAX-2);
    inheritance = *getTerm(TERMS_MAX-3);
}

