#include "Encode.h"

int term_index = 0;
Term Encode_Term(char *name)
{
    char number = -1;
    for(char i=0; i<term_index; i++)
    {
        if(!strcmp(terms[i], name))
        {
            number = i+1;
            break;
        }
    }
    if(number == -1)
    {
        assert(term_index < 255, "Too many terms for YAN");
        number = term_index+1;
        terms[term_index] = name;
        term_index++;
    }
    Term ret = {0};
    ret.terms[0] = number;
    assert(ret.terms[0] > 0, "issue with encoding in Encode_Term");
    return ret;
}
