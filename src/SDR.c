#include "SDR.h"


int SDR_ReadBitInBlock(SDR *sdr, int block_i, int block_bit_j)
{
    return (sdr->blocks[block_i] >> block_bit_j) & 1;
}

void SDR_WriteBitInBlock(SDR *sdr, int block_i, int block_bit_j, int value)
{
    sdr->blocks[block_i] = (sdr->blocks[block_i] & (~(1 << block_bit_j))) | (value << block_bit_j);
}

int SDR_ReadBit(SDR *sdr, int bit_i)
{
    SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i)
    return SDR_ReadBitInBlock(sdr, block_i, block_bit_i);
}

void SDR_WriteBit(SDR *sdr, int bit_i, int value)
{
    SDR_INDEX_TO_BLOCK_AND_BIT(bit_i, block_i,block_bit_i)
    SDR_WriteBitInBlock(sdr, block_i, block_bit_i, value);
}

void SDR_PrintFull(SDR *sdr)
{
    ITERATE_SDR_BITS(i,j,
        printf("%d,",(int) SDR_ReadBitInBlock(sdr,i,j));
    )
    printf("\n");
}

void SDR_PrintWhereTrue(SDR *sdr) {
    ITERATE_SDR_BITS(i,j,
        if (SDR_ReadBitInBlock(sdr,i,j)) {
            printf("(%d,%d)\n", i, j);
        }
    )
    printf("===\n");
}

SDR SDR_Minus(SDR a, SDR b)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = a.blocks[i] & (~b.blocks[i]);
    )
    return c;
}

SDR SDR_Union(SDR a, SDR b)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = a.blocks[i] | b.blocks[i];
    )
    return c;
}

SDR SDR_Intersection(SDR a, SDR b)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = a.blocks[i] & b.blocks[i];
    )
    return c;
}

SDR SDR_Xor(SDR a, SDR b)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = a.blocks[i] ^ b.blocks[i];
    )
    return c;
}

//permutation for sequence encoding
int seq_permutation[SDR_SIZE];
int seq_permutation_inverse[SDR_SIZE];
void initSequPermutation()
{
    for(int i=0; i<=SDR_SIZE-2; i++)
    {
        //choose an random integer so that 0<=i<=j<=SDR_SIZE
        int j = i+(random() % (SDR_SIZE-i));
        seq_permutation[i] = j;
        seq_permutation_inverse[j] = i;
    }
}

void swap(SDR *sdr, int bit_i, int bit_j)
{
    //temp <- a, then a <- b, then b <- temp
    int temp = SDR_ReadBit(sdr, bit_i);
    SDR_WriteBit(sdr, bit_i, SDR_ReadBit(sdr, bit_j));
    SDR_WriteBit(sdr, bit_j, temp);
}

SDR SDR_Copy(SDR original)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = original.blocks[i];
    )
    return c;
}
SDR SDR_Permute(SDR sdr, bool forward)
{
    SDR c = SDR_Copy(sdr);
    for(int i=0; i<SDR_SIZE; i++)
    {
        swap(&c, i, forward ? seq_permutation[i] : seq_permutation_inverse[i]);
    }
    return c;
}

SDR SDR_Set(SDR a, SDR b)
{
    return SDR_Union(a, b);
}

SDR SDR_Tuple(SDR *a, SDR *b)
{
    SDR aPerm = SDR_Permute(*a,true);
    return SDR_Xor(aPerm,*b);    
}

SDR SDR_TupleGetFirstElement(SDR *compound, SDR *secondElement)
{
    SDR aPerm = SDR_Permute(*secondElement, true);
    return SDR_Xor(aPerm, *compound);
}

SDR SDR_TupleGetSecondElement(SDR *compound, SDR *firstElement)
{
	SDR aPerm = SDR_Xor(*firstElement,*compound);
	return SDR_Permute(aPerm, false);

}
double SDR_Match(SDR *part,SDR *full)
{
    int countOneInBoth = 0;
    int countOneInPart = 0;
    ITERATE_SDR_BITS(i,j,
        countOneInBoth += (SDR_ReadBitInBlock(part,i,j) & SDR_ReadBitInBlock(full,i,j));
        countOneInPart += SDR_ReadBitInBlock(part,i,j);
    )
    return (double)countOneInBoth / countOneInPart;
}

double SDR_Inheritance(SDR full, SDR part)
{
    return SDR_Match(&part, &full);
}

double SDR_Similarity(SDR a, SDR b)
{
    return SDR_Match(&a, &b) * SDR_Match(&b, &a);
}

void SDR_INIT()
{
    initSequPermutation();
}

