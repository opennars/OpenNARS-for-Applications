/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "InvertedAtomIndex.h"

ConceptChainElement* conceptChainElementStoragePointers[UNIFICATION_DEPTH*CONCEPTS_MAX];
ConceptChainElement conceptChainElementStorage[UNIFICATION_DEPTH*CONCEPTS_MAX];
Stack conceptChainElementStack;
ConceptChainElement *invertedAtomIndex[ATOMS_MAX];

void InvertedAtomIndex_INIT()
{
    for(int i=0; i<ATOMS_MAX; i++)
    {
        invertedAtomIndex[i] = NULL;
    }
    Stack_INIT(&conceptChainElementStack, (void**) conceptChainElementStoragePointers, UNIFICATION_DEPTH*CONCEPTS_MAX);
    for(int i=0; i<UNIFICATION_DEPTH*CONCEPTS_MAX; i++)
    {
        conceptChainElementStorage[i] = (ConceptChainElement) {0};
        conceptChainElementStoragePointers[i] = NULL;
        Stack_Push(&conceptChainElementStack, &conceptChainElementStorage[i]);
    }
}

void InvertedAtomIndex_AddConcept(Term term, Concept *c)
{
    for(int i=0; i<UNIFICATION_DEPTH; i++)
    {
        Atom atom = term.atoms[i];
        if(Narsese_IsSimpleAtom(atom))
        {
            ConceptChainElement *elem = invertedAtomIndex[atom];
            if(elem == NULL)
            {
                ConceptChainElement *newElem = Stack_Pop(&conceptChainElementStack); //new item
                newElem->c = c;
                invertedAtomIndex[atom] = newElem;
            }
            else
            {
                //search for c:
                ConceptChainElement *previous = NULL;
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
                ConceptChainElement *newElem = Stack_Pop(&conceptChainElementStack); //new item
                newElem->c = c;
                previous->next = newElem;
            }
        }
        NEXT_ATOM:;
    }
}

void InvertedAtomIndex_RemoveConcept(Term term, Concept *c)
{
    for(int i=0; i<UNIFICATION_DEPTH; i++)
    {
        Atom atom = term.atoms[i];
        if(Narsese_IsSimpleAtom(atom))
        {
            ConceptChainElement *previous = NULL;
            ConceptChainElement *elem = invertedAtomIndex[atom];
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
                    Stack_Push(&conceptChainElementStack, elem);
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
            ConceptChainElement *elem = invertedAtomIndex[atom];
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

ConceptChainElement* InvertedAtomIndex_GetConceptChain(Atom atom)
{
    ConceptChainElement* ret = NULL;
    if(atom != 0)
    {
        ret = invertedAtomIndex[atom];
    }
    return ret;
}
