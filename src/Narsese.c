#include "Narsese.h"
#include "YAN.h"

//upper bound of multplier 3 given by [ becoming "(' " replacement
#define REPLACEMENT_LEN 3*NARSESE_LEN_MAX
//size for the expanded array with spaces for tokenization, has at most 3 times the amount of chars as the replacement array
#define EXPANSION_LEN REPLACEMENT_LEN*3

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
        if(narsese[i] == '<' && narsese[i+1] != '-') // < becomes (
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
            if(narsese[i] == '&' && narsese[i+1] == '|') // &| becomes ;
            {
                narsese_replaced[j] = ';';
                i+=2; j++;
            }
            else
            if(narsese[i] == '&' && narsese[i+1] == '&') // && becomes ;
            {
                narsese_replaced[j] = ';';
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
                if(narsese[i] == '<' && narsese[i+1] == '-' && narsese[i+2] == '>') // -<-> becomes :
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
                if(narsese[i] == '=' && narsese[i+1] == '=' && narsese[i+2] == '>') // ==> becomes $
                {
                    narsese_replaced[j] = '$';
                    i+=3; j++;
                }
                else
                if(narsese[i] == '-' && narsese[i+1] == '-' && narsese[i+2] != '>') // -- becomes !
                {
                    narsese_replaced[j] = '!';
                    i+=3; j++;
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

static char* canonical_copulas = "@*&|;:=$'\"/\\.-%#~+!";
char** Narsese_PrefixTransform(char* narsese_expanded)
{
    static char* tokens[NARSESE_LEN_MAX+1]; //there cannot be more tokens than chars
    memset(tokens, 0, (NARSESE_LEN_MAX+1)*sizeof(char*)); //and last one stays NULL for sure
    char* token = strtok(narsese_expanded, " ");
    int nt=0, nc = strlen(canonical_copulas);
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
                if(tokens[i+1][0]==(int)canonical_copulas[k] && tokens[i+1][1] == 0)
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

int operator_index = 0;
int Narsese_OperatorIndex(char *name)
{
    int ret_index = -1;
    for(int i=0; i<operator_index; i++)
    {
        if(!strcmp(Narsese_operatorNames[i], name))
        {
            ret_index = i+1;
            break;
        }
    }
    if(ret_index == -1)
    {
        assert(operator_index < OPERATIONS_MAX, "Too many operators, increase OPERATIONS_MAX!");
        ret_index = operator_index+1;
        strncpy(Narsese_operatorNames[operator_index], name, ATOMIC_TERM_LEN_MAX);
        operator_index++;
    }
    return ret_index;
}

int term_index = 0;
//Returns the memoized index of an already seen atomic term
int Narsese_AtomicTermIndex(char *name)
{
    int ret_index = -1;
    for(int i=0; i<term_index; i++)
    {
        if(!strcmp(Narsese_atomNames[i], name))
        {
            ret_index = i+1;
            break;
        }
    }
    if(name[0] == '^')
    {
        Narsese_OperatorIndex(name);
    }
    if(ret_index == -1)
    {
        assert(term_index < TERMS_MAX, "Too many terms for YAN");
        ret_index = term_index+1;
        strncpy(Narsese_atomNames[term_index], name, ATOMIC_TERM_LEN_MAX);
        term_index++;
    }
    return ret_index;
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
            bintree->atoms[tree_index-1] = Narsese_AtomicTermIndex("@"); //just use "@" for second element as terminator, while "." acts for "deeper" sets than 2
        }
    }
}

Term Narsese_Term(char *narsese)
{
    Term ret = {0};
    char *narsese_expanded = Narsese_Expand(narsese);
    char** tokens_prefix = Narsese_PrefixTransform(narsese_expanded);
    int nt = 0; for(;tokens_prefix[nt] != NULL; nt++){}
    buildBinaryTree(&ret, tokens_prefix, 0, 1, nt);
    return ret;
}

void Narsese_TermPunctEventTv(char *narsese, Term *destTerm, char *punctuation, bool *isEvent, Truth *destTv) {
    char narseseInplace[NARSESE_LEN_MAX];

    destTv->frequency = YAN_DEFAULT_FREQUENCY;
    destTv->confidence = YAN_DEFAULT_CONFIDENCE;

    memcpy(narseseInplace, narsese, NARSESE_LEN_MAX);
    
    int len = strlen(narseseInplace);
    if (len == 0)
    {
        return;
    }

    len = MIN(len, NARSESE_LEN_MAX); //avoid

    //tv is present if last letter is '}'
    if (narseseInplace[len-1] == '}')
    {
        //scan for opening '{'
        int openingIdx;
        for(openingIdx=len-2;openingIdx>=0;openingIdx--)
        {
            if (narseseInplace[openingIdx] == '{')
            {
                break; //found
            }
        }
        if (narseseInplace[openingIdx] == '{')
        { //was found?
            //parse TV
            double conf, freq;
            sscanf(&narseseInplace[openingIdx], "{%lf %lf}", &freq, &conf);
            destTv->frequency = freq;
            destTv->confidence = conf;

            narseseInplace[openingIdx] = 0; //cut it away for further parsing of term
        }
        else {
            //parsing error
            //we don't have a way to signal parsing error, so we pretend that nothing happened
        }
    }
    
    //parse event marker, punctuation, and finally the term:
    int str_len = strlen(narseseInplace);
    *isEvent = str_len >= 3 && narseseInplace[str_len-1] == ':' && narseseInplace[str_len-2] == '|' && narseseInplace[str_len-3] == ':'; 
    int punctuation_offset = *isEvent ? 5 : 1;
    *punctuation = narseseInplace[str_len-punctuation_offset];
    assert(*punctuation == '!' || *punctuation == '?' || *punctuation == '.', "Punctuation has to be belief . goal ! or question ?");
    narseseInplace[str_len-punctuation_offset] = 0; //we will only parse the term before it
    
    if (*punctuation != '.' && *punctuation != '?' && *punctuation != '!') {
        *punctuation = '.'; //no way to signal parsing error, force it to be a judgement as a fallback
    }

    *destTerm = Narsese_Term(narseseInplace);
}

Term Narsese_Sequence(Term *a, Term *b)
{
    Term ret = {0};
    ret.atoms[0] = Narsese_AtomicTermIndex("+");
    Term_OverrideSubterm(&ret,1,a);
    Term_OverrideSubterm(&ret,2,b);
    return ret;
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
        if(Narsese_copulaEquals(atom, ':'))
        {
            fputs("-->", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, '$'))
        {
            fputs("=/>", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, '+'))
        {
            fputs("&/", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, ';'))
        {
            fputs("&|", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, '='))
        {
            fputs("<->", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, '/'))
        {
            fputs("/1", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, '%'))
        {
            fputs("/2", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, '\\'))
        {
            fputs("\\1", stdout);
        }
        else
        if(Narsese_copulaEquals(atom, '#'))
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
    bool hasRightChild = child2 < COMPOUND_TERM_SIZE_MAX && term->atoms[child2-1] && !Narsese_copulaEquals(term->atoms[child2-1], '@');
    bool isExtSet = Narsese_copulaEquals(atom, '"');
    bool isIntSet = Narsese_copulaEquals(atom, '\'');
    bool isStatement = Narsese_copulaEquals(atom, '$') || Narsese_copulaEquals(atom, ':') || Narsese_copulaEquals(atom, '=');
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
    }
    if(child1 < COMPOUND_TERM_SIZE_MAX)
    {
        Narsese_PrintTermPrettyRecursive(term, child1);
    }
    if(hasRightChild)
    {
        fputs(hasLeftChild ? " " : "", stdout);
    }
    if(!isExtSet && !isIntSet && !Narsese_copulaEquals(atom, '@'))
    {
        Narsese_PrintAtom(atom);
        fputs(hasLeftChild ? " " : "", stdout);
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

Atom SELF; //avoids strcmp for checking operator format
void Narsese_INIT()
{
    operator_index = term_index = 0;
    for(int i=0; i<TERMS_MAX; i++)
    {
        memset(&Narsese_atomNames[i], 0, ATOMIC_TERM_LEN_MAX);
    }
    for(int i=0; i<OPERATIONS_MAX; i++)
    {
        memset(&Narsese_operatorNames[i], 0, ATOMIC_TERM_LEN_MAX);
    }
    //index the copulas at first, to make sure these will have same index on next run
    for(int i=0; i<(int) strlen(canonical_copulas); i++)
    {
        char cop[2] = {canonical_copulas[i], 0};
        Narsese_AtomicTermIndex(cop);
    }
    SELF = Narsese_AtomicTermIndex("SELF");
    Narsese_AtomicTermIndex("$1");
    Narsese_AtomicTermIndex("$2");
}

bool Narsese_copulaEquals(Atom atom, char name)
{
    return Narsese_atomNames[(int) atom-1][0] == name && Narsese_atomNames[(int) atom-1][1] == 0;
}

bool Narsese_isOperator(Atom atom)
{
    return Narsese_atomNames[(int) atom-1][0] == '^';
}

bool Narsese_isOperation(Term *term) //<(*,{SELF},x) --> ^op> -> [: * ^op " x _ _ SELF] or simply ^op
{
    return Narsese_isOperator(term->atoms[0]) ||
           (Narsese_copulaEquals(term->atoms[0], ':') && Narsese_copulaEquals(term->atoms[1], '*') && //(_ * _) -->
            Narsese_isOperator(term->atoms[2]) && //^op
            Narsese_copulaEquals(term->atoms[3], '"') && term->atoms[7] == SELF); //  { SELF }
}

int Narsese_getOperationID(Term *term)
{
    if(Narsese_copulaEquals(term->atoms[0], '+')) //sequence
    {
        Term potential_operator = Term_ExtractSubterm(term, 2); //(a &/ ^op)
        assert(!Narsese_copulaEquals(potential_operator.atoms[0], '+'), "Sequences should be left-nested encoded, never right-nested!!");
        return Narsese_getOperationID(&potential_operator);
    }
    if(Narsese_isOperator(term->atoms[0])) //atomic operator
    {
        return Narsese_OperatorIndex(Narsese_atomNames[(int) term->atoms[0]-1]);
    }
    if(Narsese_isOperation(term)) //an operation, we use the operator atom's index on the right side of the inheritance
    {
        return Narsese_OperatorIndex(Narsese_atomNames[(int) term->atoms[2]-1]);
    }
    return 0; //not an operation term
}

Term Narsese_GetPreconditionWithoutOp(Term *precondition)
{
    if(Narsese_copulaEquals(precondition->atoms[0], '+'))
    {
        Term potential_op = Term_ExtractSubterm(precondition, 2);
        if(Narsese_isOperation(&potential_op))
        {
            return Term_ExtractSubterm(precondition, 1);
        }
    }
    return *precondition;
}

bool Narsese_IsNonCopulaAtom(Atom atom)
{
    return (Narsese_atomNames[(int) atom - 1][0] >= 'a' && Narsese_atomNames[(int) atom - 1][0] <= 'z') ||
           (Narsese_atomNames[(int) atom - 1][0] >= 'A' && Narsese_atomNames[(int) atom - 1][0] <= 'Z') ||
           (Narsese_atomNames[(int) atom - 1][0] >= '0' && Narsese_atomNames[(int) atom - 1][0] <= '9');
}
