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

#ifndef H_INVERTEDATOMINDEX
#define H_INVERTEDATOMINDEX

///////////////////////////
//  Inverted atom index  //
///////////////////////////
//The inverted atom table for efficient query of to an event semantically related concepts

//References//
//////////////
#include "Concept.h"
#include "Stack.h"
#include "Config.h"

//Data structure//
//--------------//
typedef struct
{
    Concept *c;
    void *next;
}ConceptChainElement;
extern ConceptChainElement* conceptChainElementStoragePointers[UNIFICATION_DEPTH*CONCEPTS_MAX];
extern ConceptChainElement conceptChainElementStorage[UNIFICATION_DEPTH*CONCEPTS_MAX];
extern Stack conceptChainElementStack;
extern ConceptChainElement *invertedAtomIndex[ATOMS_MAX];

//Methods//
//-------//
//Init inverted atom index
void InvertedAtomIndex_INIT();
//Add concept to inverted atom index
void InvertedAtomIndex_AddConcept(Term term, Concept *c);
//Remove concept from inverted atom index
void InvertedAtomIndex_RemoveConcept(Term term, Concept *c);
//Print the inverted atom index
void InvertedAtomIndex_Print();
//Get the invtable chain with the concepts for an atom
ConceptChainElement* InvertedAtomIndex_GetConceptChain(Atom atom);

#endif
