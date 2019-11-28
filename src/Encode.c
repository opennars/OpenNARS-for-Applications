#include "Encode.h"

int term_index = 0;
Term Encode_AtomicTerm(char *name)
{
    int number = -1;
    for(int i=0; i<term_index; i++)
    {
        if(!strcmp(atoms[i], name))
        {
            number = i+1;
            break;
        }
    }
    if(number == -1)
    {
        assert(term_index < 255, "Too many terms for YAN");
        number = term_index+1;
        atoms[term_index] = name;
        term_index++;
    }
    Term ret = {0};
    ret.atoms[0] = number;
    assert(ret.atoms[0] > 0, "issue with encoding in Encode_Term");
    return ret;
}

Term Encode_Term(char *name)
{
    //TODO!!!!!
    return Encode_AtomicTerm(name);
}
