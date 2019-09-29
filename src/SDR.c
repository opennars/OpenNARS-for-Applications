#include "SDR.h"

int SDR_ReadBitInBlock(SDR *sdr, int block_i, int block_bit_j)
{
    return (sdr->blocks[block_i] >> block_bit_j) & 1;
}

void SDR_WriteBitInBlock(SDR *sdr, int block_i, int block_bit_j, int value)
{
    sdr->blocks[block_i] = (sdr->blocks[block_i] & (~(((SDR_BLOCK_TYPE)1) << block_bit_j))) | (((SDR_BLOCK_TYPE)value) << block_bit_j);
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

void SDR_Print(SDR *sdr)
{
    ITERATE_SDR_BITS(i,j,
        if (SDR_ReadBitInBlock(sdr,i,j)) 
        {
            printf("[%d](%d,%d)\n", (int) (i*SDR_BLOCK_SIZE+j), i, j);
        }
    )
    puts("===");
}

int SDR_CountTrue(SDR *sdr)
{
    int cnt = 0;
    ITERATE_SDR_BITS(i,j,
        if (SDR_ReadBitInBlock(sdr,i,j)) 
        {
            cnt++;
        }
    )
    return cnt;
}

//Xor of both SDR's
static SDR SDR_Xor(SDR *a, SDR *b)
{
    SDR c;
    ITERATE_SDR_BLOCKS(i,
        c.blocks[i] = a->blocks[i] ^ b->blocks[i];
    )
    return c;
}

SDR SDR_PermuteByRotation(SDR *sdr, bool forward)
{
    SDR c = *sdr;
    int shiftToLeftmost = SDR_BLOCK_SIZE-1;
    if(forward)
    {
        for(unsigned int i=0; i<SDR_NUM_BLOCKS; i++)
        {
            SDR_BLOCK_TYPE left_bit = c.blocks[i] & ((SDR_BLOCK_TYPE)1 << shiftToLeftmost);
            c.blocks[i] = (c.blocks[i]<<1) | (left_bit > 0);
        }
    }
    else
    {
        for(unsigned int i=0; i<SDR_NUM_BLOCKS; i++)
        {
            SDR_BLOCK_TYPE right_bit = c.blocks[i] & 1;
            c.blocks[i] = (c.blocks[i]>>1) | (right_bit << shiftToLeftmost);
        }
    }
    return c;
}

//permutation for tuple encoding
int SDR_permS[SDR_SIZE];
int SDR_permS_inv[SDR_SIZE];
int SDR_permP[SDR_SIZE];
int SDR_permP_inv[SDR_SIZE];

SDR SDR_Tuple(SDR *a, SDR *b)
{
    SDR aPerm = SDR_Permute(a,SDR_permS);
    SDR bPerm = SDR_Permute(b,SDR_permP);
    return SDR_Xor(&aPerm, &bPerm);    
}

SDR SDR_TupleGetFirstElement(SDR *compound, SDR *secondElement)
{
    SDR bPerm = SDR_Permute(secondElement, SDR_permP);
    SDR sdrxor = SDR_Xor(&bPerm, compound);
    SDR a = SDR_Permute(&sdrxor, SDR_permS_inv);
    return a;
}

SDR SDR_TupleGetSecondElement(SDR *compound, SDR *firstElement)
{
    SDR aPerm = SDR_Permute(firstElement, SDR_permS);
    SDR sdrxor = SDR_Xor(&aPerm, compound);
    SDR b = SDR_Permute(&sdrxor, SDR_permP_inv);
    return b;
}

bool SDR_Equal(SDR *a, SDR *b)
{
    ITERATE_SDR_BLOCKS(i,
        if(a->blocks[i] != b->blocks[i])
        {
            return false;
        }
    )
    return true;
}

Truth SDR_Inheritance(SDR *full, SDR *part)
{
    int countOneInBoth = 0;
    int generalCaseMisses1Bit = 0;
    ITERATE_SDR_BITS(i,j,
        countOneInBoth += (SDR_ReadBitInBlock(part,i,j) & SDR_ReadBitInBlock(full,i,j));
        generalCaseMisses1Bit += (~SDR_ReadBitInBlock(part,i,j) & SDR_ReadBitInBlock(full,i,j));
    )
    double E_total = countOneInBoth + generalCaseMisses1Bit;
    double f_total = ((double) countOneInBoth)/E_total; 
    Truth truth = { .frequency = f_total, .confidence = Truth_w2c(E_total)};
    return truth;
}

Truth SDR_Similarity(SDR *a, SDR *b)
{
    return Truth_Intersection(SDR_Inheritance(a,b), SDR_Inheritance(b,a));
}

SDR_HASH_TYPE SDR_Hash(SDR *sdr)
{
    SDR_HASH_TYPE hash = 0;
    ITERATE_SDR_BLOCKS(i,
        int pieces = SDR_BLOCK_SIZE / SDR_HASH_TYPE_SIZE;
        for(int j=0; j<pieces; j++)
        {
            int shift_right = j*SDR_HASH_TYPE_SIZE; //each j shifts 8*NUM_BYTES_OF_CONCEPT_HASH_TYPE
            hash |= (sdr->blocks[i] >> shift_right);
        }
    )
    return hash;
}

void SDR_GeneratePermutation(int *perm, int *perm_inverse)
{
    for(int i=0; i<SDR_SIZE; i++)
    {
        perm[i] = i;
    }
    for(int i=0; i<=SDR_SIZE-2; i++)
    {
        //choose an random integer so that 0<=i<=j<=SDR_SIZE
        int j = i+(rand() % (SDR_SIZE-i));
        int temp = perm[i];
        perm[i] = perm[j];
        perm[j] = temp;
    }
    for(int i=0; i<SDR_SIZE; i++)
    {
        perm_inverse[perm[i]] = i;
    }
}

SDR SDR_Permute(SDR *sdr, int *permutation)
{
    SDR c = {0};
    for(int i=0; i<SDR_SIZE; i++)
    {
        if(SDR_ReadBit(sdr, i))
        {
            SDR_WriteBit(&c, permutation[i], 1);
        }
    }
    return c;
}

void SDR_INIT()
{
    SDR_GeneratePermutation(SDR_permS, SDR_permS_inv);
    SDR_GeneratePermutation(SDR_permP, SDR_permP_inv);
}
