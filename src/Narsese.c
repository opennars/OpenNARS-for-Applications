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

#include "Narsese.h"
#include "NAR.h"

//Atomic term names:
char Narsese_atomNames[ATOMS_MAX][ATOMIC_TERM_LEN_MAX];
char Narsese_operatorNames[OPERATIONS_MAX][ATOMIC_TERM_LEN_MAX];
//upper bound of multplier 3 given by [ becoming "(' " replacement
#define REPLACEMENT_LEN 3*NARSESE_LEN_MAX
//size for the expanded array with spaces for tokenization, has at most 3 times the amount of chars as the replacement array
#define EXPANSION_LEN REPLACEMENT_LEN*3
//whether the package is initialized
static bool initialized = false;
//SELF atom, avoids strcmp for checking operator format
Atom SELF;

//Replace copulas with canonical single-char copulas, including sets and set elements!
char* replaceWithCanonicalCopulas(char *narsese, int n)
{
    static char narsese_replaced[REPLACEMENT_LEN]; 
    memset(narsese_replaced, ' ', REPLACEMENT_LEN);
    //upper bound of 3 increment given by max. look-ahead of --> becoming :, plus 0 at the end
    assert(n+3 <= NARSESE_LEN_MAX, "NARSESE_LEN_MAX too small, consider increasing or split input into multiple statements! \n");
    int j=0;
    for(int i=0; i<n; )
    {
        if(narsese[i] == ',') //, becomes " "
        {
            narsese_replaced[j] = ' '; 
            i++; j++;
        }
        else
        if(narsese[i] == '[') // [ becomes "(' "
        {
            narsese_replaced[j] = '('; 
            narsese_replaced[j+1] = '\''; 
            narsese_replaced[j+2] = ' '; 
            i++; j+=3;
        }
        else
        if(narsese[i] == '{') // { becomes '(" '
        {
            narsese_replaced[j] = '('; 
            narsese_replaced[j+1] = '"'; 
            narsese_replaced[j+2] = ' '; 
            i++; j+=3;
        }
        else
        if(narsese[i] == '}' || narsese[i] == ']' || narsese[i] == '>') // }, ], > becomes )
        {
            narsese_replaced[j] = ')';
            i++; j++; 
        }
        else
        if(narsese[i] == '<' && narsese[i+1] != '-' && narsese[i+1] != '=') // < becomes (
        {
            narsese_replaced[j] = '(';
            i++; j++; 
        }
        else
        if(i+1 < n)
        {
            if(narsese[i] == '&' && narsese[i+1] == '/') // &/ becomes +
            {
                narsese_replaced[j] = '+';
                i+=2; j++;
            }
            else
            if(narsese[i] == '&' && narsese[i+1] == '&') // && becomes ;
            {
                narsese_replaced[j] = ';';
                i+=2; j++;
            }
            else
            if(narsese[i] == '|' && narsese[i+1] == '|') // || becomes _
            {
                narsese_replaced[j] = '_';
                i+=2; j++;
            }
            else
            if(narsese[i] == '/' && narsese[i+1] == '1') // /1 becomes /
            {
                narsese_replaced[j] = '/';
                i+=2; j++;
            }
            else
            if(narsese[i] == '/' && narsese[i+1] == '2') // /2 becomes %
            {
                narsese_replaced[j] = '%';
                i+=2; j++;
            }
            else
            if(narsese[i] == '\\' && narsese[i+1] == '1') // \1 becomes backslash
            {
                narsese_replaced[j] = '\\';
                i+=2; j++;
            }
            else
            if(narsese[i] == '\\' && narsese[i+1] == '2') // \2 becomes #
            {
                narsese_replaced[j] = '#';
                i+=2; j++;
            }
            else
            if(i+2 < n)
            {
                if(narsese[i] == '-' && narsese[i+1] == '-' && narsese[i+2] == '>') // --> becomes :
                {
                    narsese_replaced[j] = ':';
                    i+=3; j++;
                }
                else
                if(narsese[i] == '|' && narsese[i+1] == '-' && narsese[i+2] == '>') // |-> becomes ,
                {
                    narsese_replaced[j] = ',';
                    i+=3; j++;
                }
                else
                if(narsese[i] == '<' && narsese[i+1] == '-' && narsese[i+2] == '>') // <-> becomes =
                {
                    narsese_replaced[j] = '=';
                    i+=3; j++;
                }
                else
                if(narsese[i] == '=' && narsese[i+1] == '/' && narsese[i+2] == '>') // =/> becomes $
                {
                    narsese_replaced[j] = '$';
                    i+=3; j++;
                }
                else
                if(narsese[i] == '=' && narsese[i+1] == '=' && narsese[i+2] == '>') // ==> becomes ?
                {
                    narsese_replaced[j] = '?';
                    i+=3; j++;
                }
                else
                if(narsese[i] == '<' && narsese[i+1] == '=' && narsese[i+2] == '>') // <=> becomes ^
                {
                    narsese_replaced[j] = '^';
                    i+=3; j++;
                }
                else
                if(narsese[i] == '-' && narsese[i+1] == '-' && narsese[i+2] != '>') // -- becomes !
                {
                    narsese_replaced[j] = '!';
                    i+=2; j++;
                }
                else
                {
                    goto DEFAULT;
                }
            }
            else
            {
                goto DEFAULT;
            }
        }
        else
        {
            goto DEFAULT;
        }
        continue;
        DEFAULT:
        narsese_replaced[j] = narsese[i]; //default
        i++; j++;
    }
    narsese_replaced[j] = 0;
    return narsese_replaced;
}

char* Narsese_Expand(char *narsese)
{
    //upper bound being 3* the multiplier of the previous upper bound
    static char narsese_expanded[EXPANSION_LEN]; 
    memset(narsese_expanded, ' ', EXPANSION_LEN);
    char *narsese_replaced = replaceWithCanonicalCopulas(narsese, strlen(narsese));
    int k = 0, n = strlen(narsese_replaced);
    for(int i=0; i<n; i++)
    {
        bool opener_closer = false;
        if(narsese_replaced[i] == '(' || narsese_replaced[i] == ')')
        {
            k+=1;
            opener_closer = true;
        }
        narsese_expanded[k] = narsese_replaced[i];
        if(narsese_replaced[i] == 0)
        {
            break;
        }
        if(opener_closer)
        {
            k+=1;
        }
        k+=1;
    }
    narsese_expanded[k] = 0;
    return narsese_expanded;
}

//skips one compound term by counting parenthesis (1 for "(", -1 for ")") till counter becomes zero again, returning the component index right after it
int skipCompound(char** tokens, int i, int nt)
{
    int parenthesis_cnt = 0; //incremented for each (, decremented for each ), count will be zero after the compound again
    for(; i<nt; i++)
    {
        if(tokens[i][0] == '(' && tokens[i][1] == 0)
        {
            parenthesis_cnt += 1;
        }
        if(tokens[i][0] == ')' && tokens[i][1] == 0)
        {
            parenthesis_cnt -= 1;
        }
        if(parenthesis_cnt==0)
        {
            return i+1;
        }
    }
    return i;
}

char** Narsese_PrefixTransform(char* narsese_expanded)
{
    static char* tokens[NARSESE_LEN_MAX+1]; //there cannot be more tokens than chars
    memset(tokens, 0, (NARSESE_LEN_MAX+1)*sizeof(char*)); //and last one stays NULL for sure
    char* token = strtok(narsese_expanded, " ");
    int nt = 0, nc = NUM_ELEMENTS(Naresese_CanonicalCopulas) - 1;
    while(token)
    {
        tokens[nt] = token;
        token = strtok(NULL, " ");
        nt++;
    }
    for(int i=0; i<nt-2; i++)
    {
        if(tokens[i][0] == '(' && tokens[i][1] == 0)   //see if it's in prefix form
        {
            for(int k=0; k<nc; k++)
            {
                if(tokens[i+1][0] == (int) Naresese_CanonicalCopulas[k] && tokens[i+1][1] == 0)
                {
                    goto Continue;
                }
            }
            //it's not a copula, so its in infix form, we need to find its copula and put it before tokens[i+1]
            int i2 = skipCompound(tokens, i+1, nt);
            if(i2 < nt)
            {
                //1. backup copula token
                char copula = tokens[i2][0];
                //2. shift all tokens forward up to copula position and set the copula to be at i+1 instead
                for(int j=i2; j>=i+2; j--)
                {
                    char *temp = tokens[j];
                    tokens[j] = tokens[j-1];
                    tokens[j-1] = temp;
                }
                tokens[i+1][0] = copula;
                tokens[i+1][1] = 0;
            }
        }
        Continue:;
    }
    return tokens;
}

HashTable HTatoms;
VMItem* HTatoms_storageptrs[ATOMS_MAX];
VMItem HTatoms_storage[ATOMS_MAX];
VMItem* HTatoms_HT[ATOMS_HASHTABLE_BUCKETS];
int term_index = 0;

//Returns the memoized index of an already seen atomic term
int Narsese_AtomicTermIndex(char *name)
{
    char blockname[ATOMIC_TERM_LEN_MAX] = {0};
    strncpy(blockname, name, ATOMIC_TERM_LEN_MAX-1);
    long ret_index = -1;
    void* retptr = HashTable_Get(&HTatoms, blockname);
    if(retptr != NULL)
    {
        ret_index = (long) retptr; //we got the value
    }
    if(ret_index == -1)
    {
        assert(term_index < ATOMS_MAX, "Too many terms for NAR");
        ret_index = term_index+1;
        strncpy(Narsese_atomNames[term_index], name, ATOMIC_TERM_LEN_MAX-1);
        HashTable_Set(&HTatoms, (void*) Narsese_atomNames[term_index], (void*) ret_index);
        term_index++;
    }
    return ret_index;
}

int Narsese_CopulaIndex(char name)
{
    char copname[2] = {0};
    copname[0] = name;
    return Narsese_AtomicTermIndex(copname);
}

//Encodes a binary tree in an array, based on the the S-expression tokenization with prefix order
void buildBinaryTree(Term *bintree, char** tokens_prefix, int i1, int tree_index, int nt)
{
    if(tokens_prefix[i1][0] == '(' && tokens_prefix[i1][1] == 0)
    {
        int icop = i1+1;
        //first argument is given by the copula
        i1 = i1+2;
        //second argument has to be searched for
        int i2 = skipCompound(tokens_prefix, i1, nt);
        bintree->atoms[tree_index-1] = Narsese_AtomicTermIndex(tokens_prefix[icop]);
        if(i1<nt)
        {
            buildBinaryTree(bintree, tokens_prefix, i1, tree_index*2, nt); //left child of tree index
        }
        if(i2<nt)
        {
            buildBinaryTree(bintree, tokens_prefix, i2, tree_index*2+1, nt); //right child of tree index
        }
    }
    else
    {
        assert(tree_index-1 < COMPOUND_TERM_SIZE_MAX, "COMPOUND_TERM_SIZE_MAX too small, consider increasing or split input into multiple statements!");
        if(!(tokens_prefix[i1][0] == ')' && tokens_prefix[i1][1] == 0))
        {
            bintree->atoms[tree_index-1] = Narsese_AtomicTermIndex(tokens_prefix[i1]);
        }
        else
        {
            bintree->atoms[tree_index-1] = Narsese_CopulaIndex(SET_TERMINATOR); //just use "@" for second element as terminator, while "." acts for "deeper" sets than 2
        }
    }
}

Term Narsese_Term(char *narsese)
{
    assert(initialized, "Narsese not initialized, call Narsese_INIT first!");
    //parse Narsese by expanding it, bringing into prefix form, then building a binary tree, and normalizing variables
    Term ret = {0};
    char *narsese_expanded = Narsese_Expand(narsese);
    char** tokens_prefix = Narsese_PrefixTransform(narsese_expanded);
    int nt = 0; for(;tokens_prefix[nt] != NULL; nt++){}
    buildBinaryTree(&ret, tokens_prefix, 0, 1, nt);
    Variable_Normalize(&ret);
    Term_Hash(&ret);
    return ret;
}

void Narsese_Sentence(char *narsese, Term *destTerm, char *punctuation, int *tense, Truth *destTv, double *occurrenceTimeOffset)
{
    assert(initialized, "Narsese not initialized, call Narsese_INIT first!");
    //Handle optional dt=num at beginning of line
    *occurrenceTimeOffset = 0.0;
    char dt[30];
    if(narsese[0] == 'd' && narsese[1] == 't'  && narsese[2] == '=') //dt=
    {
        for(unsigned int i=0; i<strlen(narsese); i++)
        {
            if(i>=3)
            {
                dt[i-3] = narsese[i];
            }
            if(narsese[i] == ' ')
            {
                dt[i] = 0;
                narsese = &narsese[i];
                break;
            }
        }
        *occurrenceTimeOffset = atof(dt);
    }
    //Handle the rest of the Narsese:
    char narseseInplace[NARSESE_LEN_MAX] = {0};
    destTv->frequency = NAR_DEFAULT_FREQUENCY;
    destTv->confidence = NAR_DEFAULT_CONFIDENCE;
    int len = strlen(narsese);
    assert(len > 1, "Parsing error: Narsese string too short!");
    assert(len < NARSESE_LEN_MAX, "Parsing error: Narsese string too long!"); //< because of '0' terminated strings
    memcpy(narseseInplace, narsese, len);
    //tv is present if last letter is '}'
    bool oldFormat = len>=2 && narseseInplace[len-1] == '%';
    if(len>=2 && (narseseInplace[len-1] == '}' || oldFormat))
    {
        //scan for opening '{'
        int openingIdx;
        bool hasComma = false;
        for(openingIdx=len-2; openingIdx>=0 && narseseInplace[openingIdx] != '{' && narseseInplace[openingIdx] != '%'; openingIdx--)
        {
            hasComma = hasComma || narseseInplace[openingIdx] == ';';
        }
        assert(narseseInplace[openingIdx] == '{' || narseseInplace[openingIdx] == '%', "Parsing error: Truth value opener not found!");
        double conf, freq;
        if(oldFormat && !hasComma)
        {
            conf = NAR_DEFAULT_CONFIDENCE;
            sscanf(&narseseInplace[openingIdx], "%%%lf%%", &freq);
        }
        else
        {
            sscanf(&narseseInplace[openingIdx], oldFormat ? "%%%lf;%lf%%" : "{%lf %lf}", &freq, &conf);
        }
        destTv->frequency = freq;
        destTv->confidence = conf;
        assert(narseseInplace[openingIdx-1] == ' ', "Parsing error: Space before truth value required!");
        narseseInplace[openingIdx-1] = 0; //cut it away for further parsing of term
    }
    //parse event marker, punctuation, and finally the term:
    int str_len = strlen(narseseInplace);
    *tense = 0;
    if(str_len >= 3 && narseseInplace[str_len-1] == ':' && narseseInplace[str_len-2] == '|' && narseseInplace[str_len-3] == ':')
        *tense = 1;
    if(str_len >= 3 && narseseInplace[str_len-1] == ':' && narseseInplace[str_len-2] == '\\' && narseseInplace[str_len-3] == ':')
        *tense = 2;
    if(str_len >= 3 && narseseInplace[str_len-1] == ':' && narseseInplace[str_len-2] == '/' && narseseInplace[str_len-3] == ':')
        *tense = 3; 
    int punctuation_offset = *tense ? 5 : 1;
    *punctuation = narseseInplace[str_len-punctuation_offset];
    assert(*punctuation == '!' || *punctuation == '?' || *punctuation == '.', "Parsing error: Punctuation has to be belief . goal ! or question ?");
    narseseInplace[str_len-punctuation_offset] = 0; //we will only parse the term before it
    *destTerm = Narsese_Term(narseseInplace);
}

Term Narsese_Sequence(Term *a, Term *b, bool *success)
{
    Term ret = {0};
    ret.atoms[0] = Narsese_CopulaIndex(SEQUENCE);
    *success = Term_OverrideSubterm(&ret,1,a) && Term_OverrideSubterm(&ret,2,b);
    return *success ? ret : (Term) {0};
}

Term Narsese_AtomicTerm(char *name)
{
    int number = Narsese_AtomicTermIndex(name);
    Term ret = {0};
    ret.atoms[0] = number;
    return ret;
}

void Narsese_PrintAtom(Atom atom)
{
    if(atom)
    {
        if(Narsese_copulaEquals(atom, INHERITANCE))
        {
            fputs("-->", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, TEMPORAL_IMPLICATION))
        {
            fputs("=/>", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, EQUIVALENCE))
        {
            fputs("<=>", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, DISJUNCTION))
        {
            fputs("||", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, SEQUENCE))
        {
            fputs("&/", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, HAS_CONTINUOUS_PROPERTY))
        {
            fputs("|->", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, IMPLICATION))
        {
            fputs("==>", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, CONJUNCTION))
        {
            fputs("&&", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, SIMILARITY))
        {
            fputs("<->", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, EXT_IMAGE1))
        {
            fputs("/1", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, EXT_IMAGE2))
        {
            fputs("/2", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, INT_IMAGE1))
        {
            fputs("\\1", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, INT_IMAGE2))
        {
            fputs("\\2", stdout);
        }
        else
        {
            fputs(Narsese_atomNames[atom-1], stdout);
        }
    }
    else
    {
        fputs("@", stdout);
    }
}

void Narsese_PrintTermPrettyRecursive(Term *term, int index) //start with index=1!
{
    Atom atom = term->atoms[index-1];
    if(!atom)
    {
        return;
    }
    int child1 = index*2;
    int child2 = index*2+1;
    bool hasLeftChild = child1 < COMPOUND_TERM_SIZE_MAX && term->atoms[child1-1];
    bool hasRightChild = child2 < COMPOUND_TERM_SIZE_MAX && term->atoms[child2-1] && !Narsese_copulaEquals(term->atoms[child2-1], SET_TERMINATOR);
    bool isSingularProduct = Narsese_copulaEquals(atom, PRODUCT) && !hasRightChild;
    bool isNegation = Narsese_copulaEquals(atom, NEGATION);
    bool isFrequencyGreater = Narsese_copulaEquals(atom, SEQUENCE) && !hasRightChild;
    bool isFrequencyEqual = Narsese_copulaEquals(atom, SIMILARITY) && !hasRightChild;
    bool isExtSet = Narsese_copulaEquals(atom, EXT_SET);
    bool isIntSet = Narsese_copulaEquals(atom, INT_SET);
    bool isStatement = !isFrequencyEqual && (Narsese_copulaEquals(atom, TEMPORAL_IMPLICATION) || Narsese_copulaEquals(atom, INHERITANCE) || Narsese_copulaEquals(atom, SIMILARITY) ||
                       Narsese_copulaEquals(atom, IMPLICATION) || Narsese_copulaEquals(atom, EQUIVALENCE) || Narsese_copulaEquals(atom, HAS_CONTINUOUS_PROPERTY));
    if(isExtSet)
    {
        fputs(hasLeftChild ? "{" : "", stdout);
    }
    else
    if(isIntSet)
    {
        fputs(hasLeftChild ? "[" : "", stdout);
    }
    else
    if(isStatement)
    {
        fputs(hasLeftChild ? "<" : "", stdout);
    }
    else
    {
        fputs(hasLeftChild ? "(" : "", stdout);
        if(isNegation || isSingularProduct || isFrequencyGreater || isFrequencyEqual)
        {
            if(isFrequencyGreater)
            {
                fputs("+", stdout);
            }
            else
            if(isFrequencyEqual)
            {
                fputs("=", stdout);
            }
            else
            {
                Narsese_PrintAtom(atom);
            }
            fputs(" ", stdout);
        }
    }
    if(child1 < COMPOUND_TERM_SIZE_MAX)
    {
        Narsese_PrintTermPrettyRecursive(term, child1);
    }
    if(hasRightChild)
    {
        fputs(hasLeftChild ? " " : "", stdout);
    }
    if(!isExtSet && !isIntSet && !Narsese_copulaEquals(atom, SET_TERMINATOR))
    {
        if(!isNegation && !isSingularProduct && !isFrequencyEqual && !isFrequencyGreater)
        {
            Narsese_PrintAtom(atom);
            fputs(hasLeftChild ? " " : "", stdout);
        }
    }
    if(child2 < COMPOUND_TERM_SIZE_MAX)
    {
        Narsese_PrintTermPrettyRecursive(term, child2);
    }
    if(isExtSet)
    {
        fputs(hasLeftChild ? "}" : "", stdout);
    }
    else
    if(isIntSet)
    {
        fputs(hasLeftChild ? "]" : "", stdout);
    }
    else
    if(isStatement)
    {
        fputs(hasLeftChild ? ">" : "", stdout);
    }
    else
    {
        fputs(hasLeftChild ? ")" : "", stdout);
    }
}

void Narsese_PrintTerm(Term *term)
{
    Narsese_PrintTermPrettyRecursive(term, 1);
}

HASH_TYPE Narsese_StringHash(char *name)
{
    assert(name != NULL, "NULL ptr in Narsese_StringHash");
    char buffer[ATOMIC_TERM_LEN_MAX] = {0};
    strncpy(buffer, name, ATOMIC_TERM_LEN_MAX-1);
    int pieces = ATOMIC_TERM_LEN_MAX / HASH_TYPE_SIZE;
    assert(HASH_TYPE_SIZE*pieces == ATOMIC_TERM_LEN_MAX, "Not a multiple, issue in hash calculation (StringHash)");
    return Globals_Hash((HASH_TYPE*) buffer, pieces);
}

bool Narsese_StringEqual(char *name1, char *name2)
{
    assert(name1 != NULL && name2 != NULL, "NULL ptr in Narsese_StringEqual");
    return !strcmp(name1, name2);
}

void Narsese_INIT()
{
    HashTable_INIT(&HTatoms, HTatoms_storage, HTatoms_storageptrs, HTatoms_HT, ATOMS_HASHTABLE_BUCKETS, ATOMS_MAX, (Equal) Narsese_StringEqual, (Hash) Narsese_StringHash);
    term_index = 0;
    for(int i=0; i<ATOMS_MAX; i++)
    {
        memset(&Narsese_atomNames[i], 0, ATOMIC_TERM_LEN_MAX);
    }
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        memset(&Narsese_operatorNames[i], 0, ATOMIC_TERM_LEN_MAX);
    }
    //index variables at first, these atoms come first as also used by Substitution struct
    for(int i=1; i<=9; i++)
    {
        char dep_varname[3] = "#1";
        char indep_varname[3] = "$1";
        char query_varname[3] = "?1";
        query_varname[1] = indep_varname[1] = dep_varname[1] = (char) ('0' + i);
        Narsese_AtomicTerm(dep_varname);
        Narsese_AtomicTerm(indep_varname);
        Narsese_AtomicTerm(query_varname);
    }
    //index rule table variables next:
    for(unsigned int i=0; i<NUM_ELEMENTS(Narsese_RuleTableVars)-1; i++)
    {
        char varname[2] = " ";
        varname[0] = Narsese_RuleTableVars[i];
        Narsese_AtomicTerm(varname);
    }
    Narsese_AtomicTermIndex("Op1");
    Narsese_AtomicTermIndex("Op2");
    //index the copulas as well, to make sure these will have same index on next run
    for(int i=0; i<(int)strlen(Naresese_CanonicalCopulas); i++)
    {
        char cop[2] = { (Naresese_CanonicalCopulas[i]), 0 };
        Narsese_AtomicTermIndex(cop);
    }
    SELF = Narsese_AtomicTermIndex("SELF");
    initialized = true;
}

bool Narsese_copulaEquals(Atom atom, char name)
{
    return atom>0 && Narsese_atomNames[(int) atom-1][0] == name && Narsese_atomNames[(int) atom-1][1] == 0;
}

bool Narsese_isOperator(Atom atom)
{
    return atom>0 && Narsese_atomNames[(int) atom-1][0] == '^' && Narsese_atomNames[(int) atom-1][1];
}

bool Narsese_isOperation(Term *term) //<(*,{SELF},x) --> ^op> -> [: * ^op " x _ _ SELF] or simply ^op
{
    return Narsese_isOperator(term->atoms[0]) ||
           (Narsese_copulaEquals(term->atoms[0], INHERITANCE) && Narsese_copulaEquals(term->atoms[1], PRODUCT) && //(_ * _) -->
            Narsese_isOperator(term->atoms[2]) && //^op
            Narsese_copulaEquals(term->atoms[3], EXT_SET)); //  { SELF } or { VAR }
}

bool Narsese_isExecutableOperation(Term *term)
{
    return Narsese_isOperation(term) && (Narsese_isOperator(term->atoms[0]) || (term->atoms[7] == SELF || Variable_isVariable(term->atoms[7])));
}

Atom Narsese_getOperationAtom(Term *term)
{
    if(Narsese_copulaEquals(term->atoms[0], SEQUENCE)) //sequence
    {
        Term potential_operator = Term_ExtractSubterm(term, 2); //(a &/ ^op)
        if(Narsese_copulaEquals(potential_operator.atoms[0], SEQUENCE))
        {
            return 0;
        }
        return Narsese_getOperationAtom(&potential_operator);
    }
    if(Narsese_isOperator(term->atoms[0])) //atomic operator
    {
        return term->atoms[0];
    }
    if(Narsese_isOperation(term)) //an operation, we use the operator atom's index on the right side of the inheritance
    {
        return term->atoms[2];
    }
    return 0; //not an operation term
}

Term Narsese_getOperationTerm(Term *term)
{
    if(Narsese_copulaEquals(term->atoms[0], TEMPORAL_IMPLICATION)) //implication
    {
        Term potential_sequence = Term_ExtractSubterm(term, 1); //(a &/ ^op) =/> b
        return Narsese_getOperationTerm(&potential_sequence);
    }
    if(Narsese_copulaEquals(term->atoms[0], SEQUENCE)) //sequence
    {
        Term potential_operator = Term_ExtractSubterm(term, 2); //(a &/ ^op)
        Term potential_op_seq = {0};
        if(!Narsese_isOperation(&potential_operator) || !Narsese_OperationSequenceAppendLeftNested(&potential_op_seq, term)) //not an op, or in case there exists a, ^op such that a=(b &/ ^op) (which can happen recursively to b) extrract the op sequence
        {
            return (Term) {0};
        }
        return potential_op_seq;
    }
    if(Narsese_isOperation(term)) //operation
    {
        return *term;
    }
    return (Term) {0}; //not an operation term
}

Term Narsese_GetPreconditionWithoutOp(Term *precondition)
{
    if(Narsese_copulaEquals(precondition->atoms[0], SEQUENCE)) //&/ S P
    {
        Term potential_op = Term_ExtractSubterm(precondition, 2);
        if(Narsese_isOperation(&potential_op))
        {
            Term new_precondition = Term_ExtractSubterm(precondition, 1);
            return Narsese_GetPreconditionWithoutOp(&new_precondition);
        }
    }
    return *precondition;
}

bool Narsese_IsSimpleAtom(Atom atom)
{
    return atom > 0 && (Narsese_atomNames[(int) atom - 1][0] == '^' ||
           (Narsese_atomNames[(int) atom - 1][0] >= 'a' && Narsese_atomNames[(int) atom - 1][0] <= 'z') ||
           (Narsese_atomNames[(int) atom - 1][0] >= 'A' && Narsese_atomNames[(int) atom - 1][0] <= 'Z') ||
           (Narsese_atomNames[(int) atom - 1][0] >= '0' && Narsese_atomNames[(int) atom - 1][0] <= '9'));
}

bool Narsese_HasSimpleAtom(Term *term)
{
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(Narsese_IsSimpleAtom(term->atoms[i]))
        {
            return true;
        }
    }
    return false;
}

bool Narsese_HasOperation(Term *term)
{
    if(Narsese_getOperationAtom(term))
    {
        return true;
    }
    if(Narsese_copulaEquals(term->atoms[0], SEQUENCE))
    {
        for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
        {
            if(Narsese_isOperator(term->atoms[i]))
            {
                return true;
            }
        }
    }
    return false;
}

bool Narsese_OperationSequenceAppendLeftNested(Term *start, Term *sequence)
{
    if(Narsese_copulaEquals(sequence->atoms[0], SEQUENCE))
    {
        Term left = Term_ExtractSubterm(sequence, 1);
        Term right = Term_ExtractSubterm(sequence, 2);
        bool success1 = Narsese_OperationSequenceAppendLeftNested(start, &left);
        if(!success1)
            return false;
        bool success2 = Narsese_OperationSequenceAppendLeftNested(start, &right);
        if(!success2)
            return false;
        return true;
    }
    bool success = true;
    if(Narsese_isOperation(sequence))
    {
        if(!start->atoms[0])
        {
            *start = *sequence;
        }
        else
        {
            *start = Narsese_Sequence(start, sequence, &success);
        }
    }
    return success;
}
