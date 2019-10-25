#include "Encode.h"

SDR Encode_Scalar(int min, int max, int value) //this is just for now :)
{
    SDR result = {0};
    result.terms[0] = value % TERMS_MAX;
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

SDR Encode_Term(char *name) //also this is just for now :)
{
    int number = hash((unsigned char*)name)%(TERMS_MAX-1);
    input_terms[number].terms[0] = 1+number;
    //assert(input_terms[number].terms[0] > 0, "encoding issue");
    return input_terms[number];
}
