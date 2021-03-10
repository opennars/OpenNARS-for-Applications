"""
 * The MIT License
 *
 * Copyright 2021 The OpenNARS authors.
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
 * """

# >English input channel for OpenNARS for Applications<
#  A shallow semantic parser with basic grammar learning ability
#  by using NAL REPRESENT relations.
#  Usage: python3 superlang.py [verbose] [OutputTruth]
#  where verbose lets it show what language knowledge is utilized
#  and OutputTruth passes on the calculated truth value to the output
 
import re
import sys
import time
import subprocess
import nltk as nltk
from nltk import sent_tokenize, word_tokenize
from nltk.corpus import stopwords
from nltk import WordNetLemmatizer
from nltk.corpus import wordnet

nltk.download('punkt')
nltk.download('averaged_perceptron_tagger')
nltk.download('universal_tagset')
nltk.download('wordnet')

SyntacticalTransformations = [
    #types of tuples of words with optional members
    (r" VERB_([0-9]*) VERB_([0-9]*) ", r" VERB_\1 ADJ_\2 "), #optional hack for the lousy nltk postagger (verbs don't come in succession, and DET would have been detected, so ADJ guess is better)
    (r" BE_([0-9]*) ADP_([0-9]*) ", r" ADP_\2 "), #(optional learnable)
    (r" BE_([0-9]*) ADV_VERB_([0-9]*) ", r" ADV_VERB_\2 "), #(optional learnable)
    (r" DET_([0-9]*) ", r" "), #ignore determiner
    (r" ADJ_([0-9]*) NOUN_([0-9]*) ", r" ADJ_NOUN_\2 "),
    (r" NOUN_([0-9]*) ", r" ADJ_NOUN_\1 "),
    (r" ADV_([0-9]*) VERB_([0-9]*) ", r" ADV_VERB_\2 "),
    (r" VERB_([0-9]*) ", r" ADV_VERB_\1 "),

]

TermRepresentRelations = [
    #subject, predicate, object encoding
    (r"ADJ_NOUN_([0-9]*)", "([ %s ] & %s )", (1.0, 0.99)),
    (r"ADV_VERB_([0-9]*)", "([ %s ] & %s )", (1.0, 0.99))
]

StatementRepresentRelations = [
    #clauses to Narsese:
    (r"\A(.*) IF_([0-9]*) (.*)\Z", r" < \3 =/> \1 > ", (1.0, 0.99)), #Conditional
    (r" ADJ_NOUN_([0-9]*) ADV_VERB_([0-9]*) ADJ_NOUN_([0-9]*) ADJ_NOUN_([0-9]*) ", r" <(( ADJ_NOUN_\1 * ADJ_NOUN_\3 ) * ADJ_NOUN_\4 ) --> ADV_VERB_\2 > ", (1.0, 0.99)), #SVOO
    (r" ADJ_NOUN_([0-9]*) BE_([0-9]*) ADJ_NOUN_([0-9]*) ", r" < ADJ_NOUN_\1 --> ADJ_NOUN_\3 > ", (1.0, 0.99)), #SVC
    (r" ADJ_NOUN_([0-9]*) ADV_VERB_([0-9]*) ADJ_NOUN_([0-9]*) ", r" <( ADJ_NOUN_\1 * ADJ_NOUN_\3 ) --> ADV_VERB_\2 > ", (1.0, 0.99)), #SVO
    (r" ADJ_NOUN_([0-9]*) BE_([0-9]*) ADJ_([0-9]*) ", r" < ADJ_NOUN_\1 --> [ ADJ_\3 ]> ", (1.0, 0.99)), #SVC
    (r" ADJ_NOUN_([0-9]*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" <( ADJ_NOUN_\1 * ADJ_NOUN_\3 ) --> ADP_\2 > ", (1.0, 0.99)), #S*A (part1)
    (r" ADJ_NOUN_([0-9]*) (.*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" ADJ_NOUN_\1 \2 , < ( ADJ_NOUN_\1 * ADJ_NOUN_\4 ) --> ADP_\3 > ", (1.0, 0.90)), #S*A (part2, optional learnable)
    (r" ADJ_NOUN_([0-9]*) ADV_VERB_([0-9]*) ", r" < ADJ_NOUN_\1 --> [ ADV_VERB_\2 ] > ", (1.0, 0.99)), #SV
]

#convert universal tag set to the wordnet word types
def wordnet_tag(tag):
    if tag == "ADJ":
        return wordnet.ADJ
    elif tag == "VERB":
        return wordnet.VERB
    elif tag == "NOUN":
        return wordnet.NOUN
    elif tag == 'ADV':
        return wordnet.ADV
    else:          
        return wordnet.NOUN #default

#pos-tag the words in the input sentence, and lemmatize them thereafter using Wordnet
def sentence_and_types(text):
    tokens = [word.lower() for word in word_tokenize(text) if word.isalpha()]
    wordtypes_ordered = nltk.pos_tag(tokens, tagset='universal')
    wordtypes = dict(wordtypes_ordered)
    lemma = WordNetLemmatizer()
    tokens = [lemma.lemmatize(word, pos = wordnet_tag(wordtypes[word])) for word in tokens]
    wordtypes = dict([(tokens[i], wordtypes_ordered[i][1]) for i in range(len(tokens))])
    wordtypes = {key : ("BE" if key == "be" else ("IF" if key == "if" else ("NOUN" if value=="PRON" else value))) for (key,value) in wordtypes.items()}
    indexed_wordtypes = []
    i = 0
    lasttoken = None
    for token in tokens:
        if lasttoken == None or wordtypes[lasttoken] == "NOUN" or wordtypes[token] == "ADP" or wordtypes[token] == "IF": #adjectives don't cross these
            i += 1 #each noun or new article ends previous ADJ_NOUN index
        indexed_wordtypes.append(wordtypes[token] + "_" + str(i))
        lasttoken = token
    print("//Word types: " + str(wordtypes))
    return " " + " ".join(tokens) + " ", " " + " ".join(indexed_wordtypes) + " "

#NAL Deduction truth function
def deduction(Ta, Tb):
    return [Ta[0]*Tb[0], Ta[0]*Tb[0]*Ta[1]*Tb[1]]

#Return the concrete word (compound) term
def getWordTerm(term, curTruth):
    for (schema, compound, Truth) in TermRepresentRelations:
        m = re.match(schema, term)
        if not m:
            continue
        curTruth[:] = deduction(curTruth, Truth)
        modifier = term.split("_")[0] + "_" + m.group(1)
        atomic =  term.split("_")[1] + "_" + m.group(1)
        if modifier in wordType:
            if "verbose" in sys.argv: print("// Using " + str((schema, compound, Truth))) 
            term = compound % (wordType[modifier], wordType[atomic]) 
        else:
            term = atomic
    return wordType.get(term, term)

#Apply syntactical reductions and wanted represent relations
def reduceTypetext(typetext, applyStatementRepresentRelations = False, applyTermRepresentRelations = False):
    curTruth = [1.0, 0.9]
    for i in range(len(SyntacticalTransformations)):
        for (a, b) in SyntacticalTransformations:
            typetext = re.sub(a, b, typetext)
    if applyStatementRepresentRelations:
        for (a, b, Truth) in StatementRepresentRelations:
            typetext_new = re.sub(a, b, typetext)
            if typetext_new != typetext:
                if "verbose" in sys.argv: print("// Using " + str((a, b, Truth)))
                typetext = typetext_new
                curTruth = deduction(curTruth, Truth)
        if applyTermRepresentRelations:
            typetext = " ".join([getWordTerm(x, curTruth) for x in typetext.split(" ")])
    return typetext, curTruth

#Learn grammar pattern by building correspondence between the words&types in the example sentences with the ones in the sentence which wasn't understood 
def GrammarLearning(y):
    global StatementRepresentRelations
    if not y.startswith("<") or not y.endswith(">") or (y.count("<") > 1 and not "=/>" in y): #Only if not fully encoded/valid Narsese
        print("//What? Tell \"" + sentence.strip() + "\" in simple sentences:")
        L = []
        while True:
            try:
                s = " " + input().rstrip("\n") + " "
            except:
                exit(0)
            if s.strip() == "":
                break
            L.append(sentence_and_types(s)[0])
        mapped = ",".join([reduceTypetext(" " + " ".join([typeWord.get(x) for x in part.split(" ") if x.strip() != "" and x in typeWord]) + " ")[0] for part in L])
        if mapped.strip() != "":
            REPRESENT = ( reduceTypetext(typetextReduced)[0], mapped, (1.0, 0.45))
            print("//Added REPRESENT relation: " + str(REPRESENT))
            sys.stdout.flush()
            StatementRepresentRelations = [REPRESENT] + StatementRepresentRelations
        return True
    return False

while True:
    #Get input line and forward potential command
    try:
        line = input().rstrip("\n") #"the green cat quickly eats the yellow mouse in the old house"
    except:
        exit(0)
    isQuestion = line.endswith("?")
    isCommand = line.startswith("*") or line.startswith("//") or line.isdigit() or line.startswith('(') or line.startswith('<')
    isNegated = " not " in (" " + line + " ")
    if isCommand:
        print(line)
        sys.stdout.flush()
        continue
    print("//Input sentence: " + line)
    #it's a sentence, postag and bring it into canonical representation using Wordnet lemmatizer:
    sentence = " " + line.replace("?", "").replace(".", "").replace(",", "").replace(" not ", " ") + " "
    s_and_T = sentence_and_types(sentence)
    sentence = s_and_T[0] # canonical sentence (with lemmatized words)
    typetext = s_and_T[1] #" DET_1 ADJ_1 NOUN_1 ADV_2 VERB_2 DET_2 ADJ_2 NOUN_2 ADP_3 DET_3 ADJ_3 NOUN_3 "
    wordType = dict(zip(typetext.split(" "), sentence.split(" "))) #mappings like cat -> NOUN_1
    typeWord = dict(zip(sentence.split(" "), typetext.split(" "))) #mappings like NOUN1 -> cat
    #Transformed typetext taking syntatical relations and represent relations into account:
    (typetextReduced,    _  ) = reduceTypetext(typetext)
    (typetextNarsese,    _  ) = reduceTypetext(typetext, applyStatementRepresentRelations = True)
    (typetextConcrete, Truth) = reduceTypetext(typetext, applyStatementRepresentRelations = True, applyTermRepresentRelations = True)
    print("//Lemmatized sentence: " + sentence, "\n//Typetext: " + typetext, "\n//Typetext Narsese:" + typetextNarsese)
    sys.stdout.flush()
    #Check if one of the output representations wasn't fully transformed and demands grammar learning:
    Input = True
    for y in typetextConcrete.split(" , "):
        if GrammarLearning(y.strip()):
            Input = False
    #If not we can output the Narsese events for NARS to consume:
    if Input:
        for y in typetextConcrete.split(" , "):
            TruthString = "" if "OutputTruth" not in sys.argv else " {" + str(Truth[0]) + " " + str(Truth[1]) + "}"
            statement = "(! " + y + ")" if isNegated else " " + y + " "
            print((statement.replace(" what "," ?1 ").replace(" who "," ?1 ").replace(" it ", " $1 ").strip() + ("? :|:" if isQuestion else ". :|:")) + TruthString)
            sys.stdout.flush()
