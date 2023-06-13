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
#  Usage: python3 english_to_narsese.py [verbose] [OutputTruth] [EternalOutput]
#  where verbose lets it show what language knowledge is utilized
#  and OutputTruth passes on the calculated truth value to the output
#  and EternalOutput specifies whether the output Narsese tasks should be eternal
 
import re
import sys
import time
import subprocess
import nltk as nltk
from nltk import sent_tokenize, word_tokenize
from nltk.corpus import stopwords
from nltk import WordNetLemmatizer
from nltk.corpus import wordnet

quiet = "quiet" in sys.argv
nltk.download('punkt', quiet=quiet)
nltk.download('averaged_perceptron_tagger', quiet=quiet)
nltk.download('universal_tagset', quiet=quiet)
nltk.download('wordnet', quiet=quiet)
nltk.download('omw-1.4', quiet=quiet)

SyntacticalTransformations = [
    #types of tuples of words with optional members
    (r" VERB_([0-9]*) VERB_([0-9]*) ", r" VERB_\1 ADJ_\2 "), #hack for the lousy nltk postagger (verbs don't come in succession, DET would have been detected, ADJ is better guess)
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
    (r"ADJ_NOUN_([0-9]*)", "( [ %s ] & %s )", (1.0, 0.99)),
    (r"ADV_VERB_([0-9]*)", "( [ %s ] & %s )", (1.0, 0.99))
]

AcquiredGrammar = []
StatementRepresentRelations = [
    #clauses to Narsese:
    (r"\A(.*) IF_([0-9]*) (.*)\Z", r" < \3 =/> \1 > ", (1.0, 0.99), 0), #Conditional
    (r" ADJ_NOUN_([0-9]*) ADV_VERB_([0-9]*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" < ( ADJ_NOUN_\1 * ADJ_NOUN_\4 ) --> ADV_VERB_\2+ADP_\3 > ", (1.0, 0.99), 0), #new addition for lie_in above_of etc.
    (r" ADJ_NOUN_([0-9]*) BE_([0-9]*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" < ( ADJ_NOUN_\1 * ADJ_NOUN_\4 ) --> BE_\2+ADP_\3 > ", (1.0, 0.99), 0), #new addition for lie_in above_of etc.
    (r" ADJ_NOUN_([0-9]*) BE_([0-9]*) ADJ_NOUN_([0-9]*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" < ( ADJ_NOUN_\1 * ADJ_NOUN_\5 ) --> ADJ_NOUN_\3+ADP_\4 > ", (1.0, 0.99), 0), #new addition for lie_in above_of etc.
    (r" ADJ_NOUN_([0-9]*) BE_([0-9]*) ADJ_([0-9]*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" < ( ADJ_NOUN_\1 * ADJ_NOUN_\5 ) --> ADJ_\3+ADP_\4 > ", (1.0, 0.99), 0), #new addition for larger_than etc.
    (r" ADJ_NOUN_([0-9]*) ADV_VERB_([0-9]*) ADJ_NOUN_([0-9]*) ADJ_NOUN_([0-9]*) ", r" <(( ADJ_NOUN_\1 * ADJ_NOUN_\3 ) * ADJ_NOUN_\4 ) --> ADV_VERB_\2 > ", (1.0, 0.99), 0), #SVOO
    (r" ADJ_NOUN_([0-9]*) BE_([0-9]*) ADJ_NOUN_([0-9]*) ", r" < ADJ_NOUN_\1 --> ADJ_NOUN_\3 > ", (1.0, 0.99), 0), #SVC
    (r" ADJ_NOUN_([0-9]*) ADV_VERB_([0-9]*) ADJ_NOUN_([0-9]*) ", r" <( ADJ_NOUN_\1 * ADJ_NOUN_\3 ) --> ADV_VERB_\2 > ", (1.0, 0.99), 0), #SVO
    (r" ADJ_NOUN_([0-9]*) BE_([0-9]*) ADJ_([0-9]*) ", r" < ADJ_NOUN_\1 --> [ ADJ_\3 ]> ", (1.0, 0.99), 0), #SVC
    (r" ADJ_NOUN_([0-9]*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" <( ADJ_NOUN_\1 * ADJ_NOUN_\3 ) --> ADP_\2 > ", (1.0, 0.99), 0), #S*A (part1)
    (r" ADJ_NOUN_([0-9]*) (.*) ADP_([0-9]*) ADJ_NOUN_([0-9]*) ", r" ADJ_NOUN_\1 \2 , < ( ADJ_NOUN_\1 * ADJ_NOUN_\4 ) --> ADP_\3 > ", (1.0, 0.90), 0), #S*A (part2, optional learnable)
    (r" ADJ_NOUN_([0-9]*) ADV_VERB_([0-9]*) ", r" < ADJ_NOUN_\1 --> [ ADV_VERB_\2 ] > ", (1.0, 0.99), 0), #SV
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
    tokens = [word for word in word_tokenize(text)]
    wordtypes_ordered = nltk.pos_tag(tokens, tagset='universal')
    wordtypes = dict(wordtypes_ordered)
    lemma = WordNetLemmatizer()
    #NamedEntities = {key:value for (key,value) in [(x.lower(),x) for x in tokens]}
    handleInstance = lambda word: "{"+word+"}" if word[0].isupper() else word
    tokens = [handleInstance(lemma.lemmatize(word, pos = wordnet_tag(wordtypes[word]))) for word in tokens]
    wordtypes = dict([(tokens[i], wordtypes_ordered[i][1]) for i in range(len(tokens))])
    wordtypes = {key : ("BE" if key == "be" else ("IF" if key == "if" else ("NOUN" if value=="PRON" or value=="NUM" else ("ADP" if value=="PRT" else value)))) for (key,value) in wordtypes.items()}
    indexed_wordtypes = []
    i = 0
    lasttoken = None
    for token in tokens:
        if lasttoken == None or wordtypes[lasttoken] == "NOUN" or wordtypes[token] == "ADP" or wordtypes[token] == "IF": #adjectives don't cross these
            i += 1 #each noun or new article ends previous ADJ_NOUN index
        indexed_wordtypes.append(wordtypes[token] + "_" + str(i))
        lasttoken = token
    if "verbose" in sys.argv: print("//Word types: " + str(wordtypes))
    return " " + " ".join(tokens) + " ", " " + " ".join(indexed_wordtypes) + " "

#NAL truth functions
def Truth_Deduction(Ta, Tb):
    return [Ta[0]*Tb[0], Ta[0]*Tb[0]*Ta[1]*Tb[1]]

def Truth_w2c(w):
    return w / (w + 1.0)

def Truth_c2w(c):
    return c / (1.0 - c)

def Truth_Expectation(v):
    return (v[1] * (v[0] - 0.5) + 0.5)

def Truth_Revision(v1, v2):
    (f1, c1) = v1
    (f2, c2) = v2
    w1 = Truth_c2w(c1)
    w2 = Truth_c2w(c2)
    w = w1 + w2
    return (min(1.0, (w1 * f1 + w2 * f2) / w), 
            min(0.99, max(max(Truth_w2c(w), c1), c2)))
#NAL truth functions end

#Return the concrete word (compound) term
def getWordTerm(term, curTruth, suppressOutput = True):
    for (schema, compound, Truth) in TermRepresentRelations:
        m = re.match(schema, term)
        if not m:
            continue
        curTruth[:] = Truth_Deduction(curTruth, Truth)
        modifier = term.split("_")[0] + "_" + m.group(1)
        atomic =  term.split("_")[1] + "_" + m.group(1)
        if modifier in wordType:
            if "verbose" in sys.argv and not suppressOutput: print("// Using " + str((schema, compound, Truth))) 
            term = compound % (wordType[modifier], wordType[atomic]) 
        else:
            term = atomic
    return wordType.get(term, term)

#Apply syntactical reductions and wanted represent relations
def reduceTypetext(typetext, applyStatementRepresentRelations = False, applyTermRepresentRelations = False, suppressOutput = True):
    curTruth = [1.0, 0.9]
    for i in range(len(SyntacticalTransformations)):
        for (a, b) in SyntacticalTransformations:
            typetext = re.sub(a, b, typetext)
    if applyStatementRepresentRelations:
        for (a, b, Truth, _) in AcquiredGrammar + StatementRepresentRelations:
            typetext_new = re.sub(a, b, typetext)
            if typetext_new != typetext:
                if "verbose" in sys.argv and not suppressOutput: print("// Using " + str((a, b, Truth)))
                typetext = typetext_new
                curTruth = Truth_Deduction(curTruth, Truth)
        if applyTermRepresentRelations:
            typetext = " ".join([getWordTerm(x, curTruth, suppressOutput=suppressOutput) if "+" not in x else getWordTerm(x.split("+")[0], curTruth, suppressOutput=suppressOutput)+"_"+getWordTerm(x.split("+")[1], curTruth, suppressOutput=suppressOutput) for x in typetext.split(" ")])
    return typetext, curTruth

#Learn grammar pattern by building correspondence between the words&types in the example sentences with the ones in the sentence which wasn't understood
currentTime = 0
def GrammarLearning(y = "", forced = False):
    global AcquiredGrammar, currentTime
    if forced or (not y.startswith("<") or not y.endswith(">") or (y.count("<") > 1 and not "=/>" in y)): #Only if not fully encoded/valid Narsese
        print("//What? Tell \"" + sentence.strip() + "\" in simple sentences: (newline-separated)")
        L = []
        while True:
            try:
                s = " " + input().rstrip("\n") + " "
                print("//Example input: " + s.strip() if s.strip() != "" else "//Example done.")
            except:
                exit(0)
            if s.strip() == "":
                break
            L.append(sentence_and_types(s)[0])
        mapped = ",".join([reduceTypetext(" " + " ".join([typeWord.get(x) for x in part.split(" ") if x.strip() != "" and x in typeWord]) + " ")[0] for part in L])
        if mapped.strip() != "":
            (R,mapped,T) = ( reduceTypetext(typetextReduced)[0], mapped, (1.0, 0.45))
            for i,typeword in enumerate(R.strip().split(" ")): #generalize grammar indices
                R = R.replace(typeword, "_".join(typeword.split("_")[:-1]) + "_([0-9]*)")
                mapped = mapped.replace(typeword, "_".join(typeword.split("_")[:-1])+"_\\" + str(i+1))
            for (R2,mapped2,T2,_) in AcquiredGrammar:
                if R == R2 and mapped == mapped2:
                    T = Truth_Revision(T, T2)
                    break
            print("//Induced grammar relation: " + str((R,mapped,T)))
            sys.stdout.flush()
            AcquiredGrammar.append((R,mapped,T,currentTime))
            AcquiredGrammar.sort(key=lambda T: (-Truth_Expectation(T[2]), -T[3]))
        return True
    return False

motivation = None
thinkcycles = None
eternal = False
tenseFromSentence = True
if "EternalOutput" in sys.argv:
    eternal = True
    tenseFromSentence = False
if "EventOutput" in sys.argv:
    eternal = False
    tenseFromSentence = False
while True:
    currentTime += 1
    #Get input line and forward potential command
    try:
        line = input().rstrip("\n") #"the green cat quickly eats the yellow mouse in the old house"
    except:
        exit(0)
    if len(line) == 0:
        print("\n")
    isQuestion = line.endswith("?")
    isGoal = line.endswith("!")
    isCommand = line.startswith("*") or line.startswith("//") or line.isdigit() or line.startswith('(') or line.startswith('<') or line.endswith(":|:")
    spaced_line = (" " + line.lower() + " ")
    isNegated = " not " in spaced_line or " no " in spaced_line
    if isCommand:
        if line.startswith("*eternal=false"):
            eternal = False
            continue
        if line.startswith("*eternal=true"):
            eternal = True
            continue
        if line.startswith("*motivation="):
            motivation = line.split("*motivation=")[1]
            continue
        if line.startswith("*thinkcycles="):
            thinkcycles = line.split("*thinkcycles=")[1]
            continue
        if line.startswith("*teach"):
            GrammarLearning(forced = True)
            continue
        else:
            print(line)
            sys.stdout.flush()
            continue
    if line.strip() != "": print("//Input sentence: " + line)
    #determine tense from sentence
    punctuations = [" ", "!","?"]
    tensesPast = ["previously", "before"]
    tensesPresent = ["now", "currently", "afterwards"]
    tensesFuture = ["afterwards", "later"]
    eventTenses = tensesPast + tensesPresent + tensesFuture
    isPastEvent = True in [" "+w+p in spaced_line for p in punctuations for w in tensesPast]
    isFutureEvent = True in [" "+w+p in spaced_line for p in punctuations for w in tensesFuture]
    isEvent = True in [" "+w+p in spaced_line for p in punctuations for w in eventTenses]
    if " will be " in line: #A COMMON FUTURE EXPRESSION NOT COVERED BY ABOVE
        line = line.replace(" will be ", " is ")
        isFutureEvent = True
        isEvent = True
    nonEternalMarker = ":/:" if isFutureEvent else (":\\:" if isPastEvent else ":|:")
    if tenseFromSentence:
        eternal = not isEvent
        for punc in punctuations:
            for tenseWord in eventTenses:
                if " "+tenseWord+punc in spaced_line:
                    line = ((line + " ").replace(" "+tenseWord+punc, "")).lstrip().rstrip()
    #it's a sentence, postag and bring it into canonical representation using Wordnet lemmatizer:
    sentence = " " + line.replace("!", "").replace("?", "").replace(".", "").replace(",", "").replace(" not ", " ") + " "
    s_and_T = sentence_and_types(sentence)
    sentence = s_and_T[0] # canonical sentence (with lemmatized words)
    typetext = s_and_T[1] #" DET_1 ADJ_1 NOUN_1 ADV_2 VERB_2 DET_2 ADJ_2 NOUN_2 ADP_3 DET_3 ADJ_3 NOUN_3 "
    wordType = dict(zip(typetext.split(" "), sentence.split(" "))) #mappings like cat -> NOUN_1
    typeWord = dict(zip(sentence.split(" "), typetext.split(" "))) #mappings like NOUN1 -> cat
    #Transformed typetext taking syntatical relations and represent relations into account:
    (typetextReduced,    _  ) = reduceTypetext(typetext)
    (typetextNarsese,    _  ) = reduceTypetext(typetext, applyStatementRepresentRelations = True)
    (typetextConcrete, Truth) = reduceTypetext(typetext, applyStatementRepresentRelations = True, applyTermRepresentRelations = True, suppressOutput = False)
    if "verbose" in sys.argv: print("//Lemmatized sentence: " + sentence, "\n//Typetext: " + typetext, "\n//Typetext reduced:" + typetextReduced, "\n//Typetext Narsese:" + typetextNarsese)
    sys.stdout.flush()
    #Check if one of the output representations wasn't fully transformed and demands grammar learning:
    Input = True
    typetextSplit = [x.strip() for x in typetextConcrete.split(" , ") if x.strip() != ""]
    for y in typetextSplit:
        if GrammarLearning(y):
            Input = False
            break
    #If not we can output the Narsese events for NARS to consume:
    if Input:
        for y in typetextSplit:
            TruthString = "" if "OutputTruth" not in sys.argv else " {" + str(Truth[0]) + " " + str(Truth[1]) + "}"
            statement = "(! " + y + ")" if isNegated else " " + y + " "
            punctuation = "?" if isQuestion else ("!" if isGoal else ".")
            print(statement.replace(" {What} "," ?1 ").replace("=/>", "==>").replace(" {Who} "," ?1 ").replace(" {It} ", " $1 ").replace(" what "," ?1 ").replace(" who "," ?1 ").replace(" it ", " $1 ").strip() + (punctuation + ("" if eternal else " " + nonEternalMarker)) + TruthString)
            sys.stdout.flush()
        if len(typetextSplit) > 0 and thinkcycles != None:
            print(thinkcycles, flush=True)
    if motivation != None and line.strip() != "":
        print(motivation, flush=True)
        if thinkcycles != None:
            print(thinkcycles, flush=True)
