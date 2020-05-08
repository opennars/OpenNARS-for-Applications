#include "InvertedAtomIndex.h"

void InvertedAtomIndex_INIT()
{
    for(int i=0; i<ATOMS_MAX; i++)
    {
        invertedAtomIndex[i] = NULL;
    }
    Stack_INIT(&invTableChainElementStack, (void**) invTableChainElementStoragePointers, COMPOUND_TERM_SIZE_MAX*CONCEPTS_MAX);
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX*CONCEPTS_MAX; i++)
    {
        invTableChainElementStorage[i] = (InvtableChainElement) {0};
        invTableChainElementStoragePointers[i] = NULL;
        Stack_Push(&invTableChainElementStack, &invTableChainElementStorage[i]);
    }
}

void InvertedAtomIndex_Add(Term term, Concept *c)
{
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = term.atoms[i];
        if(Narsese_IsSimpleAtom(atom))
        {
            InvtableChainElement *elem = invertedAtomIndex[atom];
            if(elem == NULL)
            {
                InvtableChainElement *newElem = Stack_Pop(&invTableChainElementStack); //new item
                newElem->c = c;
                invertedAtomIndex[atom] = newElem;
            }
            else
            {
                //search for c:
                InvtableChainElement *previous = NULL;
                while(elem != NULL)
                {
                    if(elem->c == c)
                    {
                        goto NEXT_ATOM;
                    }
                    previous = elem;
                    elem = elem->next;
                }
                //ok, we can add it as previous->next
                InvtableChainElement *newElem = Stack_Pop(&invTableChainElementStack); //new item
                newElem->c = c;
                previous->next = newElem;
            }
        }
        NEXT_ATOM:;
    }
}

void InvertedAtomIndex_Remove(Term term, Concept *c)
{
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        Atom atom = term.atoms[i];
        if(Narsese_IsSimpleAtom(atom))
        {
            InvtableChainElement *previous = NULL;
            InvtableChainElement *elem = invertedAtomIndex[atom];
            while(elem != NULL)
            {
                if(elem->c == c) //we found c in the chain, remove it
                {
                    if(previous == NULL) //item was the initial chain element, let the next element be the initial now
                    {
                        invertedAtomIndex[atom] = elem->next;
                    }
                    else //item was within the chain, relink the previous to the next of elem
                    {
                        previous->next = elem->next;
                    }
                    //push elem back to the stack, it's "freed"
                    assert(elem->c != NULL, "A null concept was in inverted atom index!");
                    elem->c = NULL;
                    elem->next = NULL;
                    Stack_Push(&invTableChainElementStack, elem);
                    goto NEXT_ATOM;
                }
                previous = elem;
                elem = elem->next;
            }
        }
        NEXT_ATOM:;
    }
}

void InvertedAtomIndex_Print()
{
    puts("printing inverted atom table content:");
    for(int i=0; i<ATOMS_MAX; i++)
    {
        Atom atom = i; //the atom is directly the value (from 0 to ATOMS_MAX)
        if(Narsese_IsSimpleAtom(atom))
        {
            InvtableChainElement *elem = invertedAtomIndex[atom];
            while(elem != NULL)
            {
                Concept *c = elem->c;
                assert(c != NULL, "A null concept was in inverted atom index!");
                Narsese_PrintAtom(atom);
                fputs(" -> ", stdout);
                Narsese_PrintTerm(&c->term);
                puts("");
                elem = elem->next;
            }
        }
    }
    puts("table print finish");
}
