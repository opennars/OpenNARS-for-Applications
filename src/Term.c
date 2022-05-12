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

#include "Term.h"

bool Term_Equal(Term *a, Term *b)
{
    if(Term_Hash(a) == Term_Hash(b))
    {
        return memcmp(a, b, sizeof(Term)) == 0;
    }
    else
    {
        return false;
    }
}

static bool Term_RelativeOverride(Term *term, int i, Term *subterm, int j)
{
    if(i >= COMPOUND_TERM_SIZE_MAX)
    {
        return false;
    }
    if(j < COMPOUND_TERM_SIZE_MAX)
    {
        term->atoms[i] = subterm->atoms[j];
        term->hashed = false;
        int left_in_subterm = (j+1)*2-1;
        if(left_in_subterm < COMPOUND_TERM_SIZE_MAX && subterm->atoms[left_in_subterm] != 0)
        {
            if(!Term_RelativeOverride(term, (i+1)*2-1, subterm, left_in_subterm))   //override left child
            {
                return false;
            }
        }
        int right_in_subterm = (j+1)*2+1-1;
        if(right_in_subterm < COMPOUND_TERM_SIZE_MAX && subterm->atoms[right_in_subterm] != 0)
        {
            if(!Term_RelativeOverride(term, (i+1)*2+1-1, subterm, right_in_subterm)) //override right child
            {
                return false;
            }
        }
    }
    return true;
}

bool Term_OverrideSubterm(Term *term, int i, Term *subterm)
{
    return Term_RelativeOverride(term, i, subterm, 0); //subterm starts at its root, but its a subterm in term at position i
}

Term Term_ExtractSubterm(Term *term, int j)
{
    Term ret = {0}; //ret is where to "write into" 
    Term_RelativeOverride(&ret, 0, term, j); //where we begin to write at root, 0 (always succeeds as we extract just a subset)
    return ret; //reading from term beginning at i
}

int Term_Complexity(Term *term)
{
    int s = 0;
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(term->atoms[i])
        {
            s += 1;
        }
    }
    return s;
}

HASH_TYPE Term_Hash(Term *term)
{
    if(term->hashed)
    {
        return term->hash;
    }
    int pieces = TERM_ATOMS_SIZE / HASH_TYPE_SIZE;
    assert(HASH_TYPE_SIZE*pieces == TERM_ATOMS_SIZE, "Not a multiple, issue in hash calculation (TermHash)");
    HASH_TYPE hash = Globals_Hash((HASH_TYPE*) term->atoms, pieces);
    term->hashed = true;
    term->hash = hash;
    return hash;
}

bool Term_HasAtom(Term *term, Atom atom)
{
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(term->atoms[i] == atom)
        {
            return true;
        }
    }
    return false;
}
