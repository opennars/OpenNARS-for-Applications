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

void Narsese_Test()
{
    puts(">>Narsese test start");
    char* narsese = "<<$sth --> (&,[furry,meowing],animal)> =/> <$sth --> [good]>>";
    printf("Narsese: %s\n", narsese);
    char* preprocessed = Narsese_Expand(narsese);
    printf("Preprocessed: %s\n", preprocessed);
    char **tokens = Narsese_PrefixTransform(preprocessed);
    int k = 0;
    for(;tokens[k] != NULL;k++)
    {
        printf("token: %s\n", tokens[k]);
    }
    Term ret = Narsese_Term(narsese);
    for(int i=0; i<COMPOUND_TERM_SIZE_MAX; i++)
    {
        if(ret.atoms[i] != 0)
        {
            printf("Subterm: %i %d %s\n", i, ret.atoms[i], Narsese_atomNames[ret.atoms[i]-1]);
        }
    }
    puts("Result:");
    Narsese_PrintTerm(&ret);
    puts("");
    puts(">>Narsese Test successul");
    Narsese_PrintTerm(&ret);
    puts("");
}
