"""
 * The MIT License
 *
 * Copyright 2023 The OpenNARS authors.
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

import sys
from Levenshtein import distance as lev
from nltk import WordNetLemmatizer
from nltk.corpus import wordnet
import openai

openai.api_key = "YOUR_KEY"
eternal = True #whether to use event or eternal output
PrintInput=False
Prepositions = False
CleanRelations = True
levenstein_threshold = 0.8
wup_threshold = 0.8
gpt_prompt = """
def PosRelation(noun,verb,noun): ... #put true relation into database
def PosRelation(noun,"IsA",noun): ... #put true category into database
def PosProperty(noun,adjective): ... #put true property into database
def NegRelation(noun,verb,noun): ... #put false relation into database
def NegRelation(noun,"IsA",noun): ... #put false category into database
def NegProperty(noun,adjective): ... #put false property into database
Capture the complete sentence meaning with code that calls the two functions, and only use a single word per argument.
The sentence: 
"""

if "EventOutput" in sys.argv:
    eternal = False

if "Prepositions" in sys.argv:
    Prepositions = True

used_verbs = set([])
lemma = WordNetLemmatizer()
def Lemmatize(word, tag, isQuestion, s=None, p=None):
    global used_verbs
    ret = lemma.lemmatize(word.lower(), pos = tag).strip().lower().replace(" ","_")
    #print("//!!!",ret)
    varwords = ["something","someone", "somewhere", "somewhen", "somewhat", "nowhere", "what", "where","which", "who", "when", "noun", "none", "unknown", "unspecified"]
    if isQuestion:
        for word in varwords:
            ret = ret.replace(word, "?1")
    if ret in used_verbs: #if already a used verb, just use it
        return ret, s, p
    if tag == wordnet.VERB: #else we search for a highly overlapping one and use that one
        #we get rid of prepositions next:
        if ret == "isa" or ret == "is_a" or ret == "be" or ret == "are":
            return "IsA", s, p
        if not Prepositions:
            prepositions_inversing = ["by"]
            for word in prepositions_inversing:
                if ret.endswith("_" + word):
                    new_s = p
                    new_p = s
                    s = new_s
                    p = new_p
                    ret = ret[:-(len(word)+1)]
                    print(f"//Args switched for relation '{ret}' due to preposition '{word}'")
                    break
            for word in ["to", "on", "in", "onto", "of", "over", "from"]:
                if "is_" + word == ret or "is" + word == ret:
                    ret = word
                elif ret.endswith("_"+word):
                    ret = ret[:-(len(word)+1)]
                    if ret == "are" or ret == "is":
                        ret = word
        if CleanRelations:
            #get rid of is_words:
            for word in ["was", "is", "be", "has", "have", "can"]:
                if ret.startswith(word+"_"):
                    ret = ret[(len(word)+1):]
            #and articles
            for word in ["the", "a"]:
                if ret.startswith(word+"_"):
                    ret = ret[(len(word)+1):]
        #replace verb with levin-similar verb
        for x in used_verbs:
            levensteinsimratio = (max(len(ret),len(x))-lev(ret, x))/max(len(ret),len(x))
            if x != ret and levensteinsimratio > levenstein_threshold:
                print(f"//using {x} instead of {ret}, lev_similarity={levensteinsimratio}")
                return x, s, p
        #replace verb with wup-similar verb
        synret = wordnet.synsets(ret, 'v')
        if synret:
            for x in used_verbs:
                synx = wordnet.synsets(x, 'v')
                if synx:
                    wupsim = synret[0].wup_similarity(synx[0])
                    if x != ret and wupsim > wup_threshold:
                        print(f"//using {x} instead of {ret}, wup_similarity={wupsim}")
                        return x, s, p
        #replace verb with simpler component
        for x in used_verbs:
            if x in ret:
                if x != ret:
                    print(f"//using {x} instead of {ret} since the former is simpler")
                    return x, s, p
        used_verbs.add(ret) #and if there is no such, use the new verb
    return ret, s, p

def Relation(s, v, p, punctuation_tv, isQuestion):
    s, _, _ = Lemmatize(s, wordnet.NOUN, isQuestion)
    p, _, _ = Lemmatize(p, wordnet.NOUN, isQuestion)
    v, s, p = Lemmatize(v, wordnet.VERB, isQuestion, s, p)
    if s == "" or v == "" or p == "":
        return
    if v == "IsA":
        print(f"<{s} --> {p}>{punctuation_tv}")
    else:
        print(f"<({s} * {p}) --> {v}>{punctuation_tv}")

def Property(s, p, punctuation_tv, isQuestion):
    s, _, _ = Lemmatize(s, wordnet.NOUN, isQuestion)
    p, _, _ = Lemmatize(p, wordnet.ADJ, isQuestion)
    if s == "" or p == "":
        return
    print(f"<{s} --> [{p}]>{punctuation_tv}")

def process_commands(commands, isQuestion):
    for x in commands:
        if (x.startswith("PosProperty(") or x.startswith("PosRelation(") or x.startswith("NegProperty(") or x.startswith("NegRelation(")) and x.endswith(")"):
            #print("//",x)
            s_v_p = x.split("(")[1].split(")")[0].replace("\"","").replace("'","").split(",")
            truth = "{1.0 0.9}"
            if x.startswith("Neg"):
                truth = "{0.0 0.9}"
            x = x[3:] #cut off Neg/Pos
            if len(s_v_p) > 3:
                continue
            eventMarker = "" if eternal else " :|:"
            punctuation_tv = f"?{eventMarker}" if isQuestion else f".{eventMarker} {truth}"
            if x.startswith("Property"):
                Property(*s_v_p, punctuation_tv, isQuestion)
            if x.startswith("Relation"):
                Relation(*s_v_p, punctuation_tv, isQuestion)

while True:
    try:
        inp = input().rstrip("\n")
        if PrintInput:
            print("//Sentence input:", inp)
    except:
        exit(0)
    if len(inp) == 0:
        print("\n")
    elif inp.startswith("*eternal=false"):
        eternal = False
    elif inp.startswith("*eternal=true"):
        eternal = True
    elif inp.isdigit() or inp.startswith("*") or inp.startswith("(") or inp.startswith("<"):
        print(inp)
    else:
        isQuestion = inp.endswith("?")
        if inp.endswith("?") or inp.endswith("."):
            inp = inp[0:-1]
        response = openai.ChatCompletion.create(
            model='gpt-3.5-turbo',
              messages=[
                {"role": "user", "content": gpt_prompt + inp}],
            max_tokens=100,
            temperature=0,
        )
        commands = response['choices'][0]['message']['content'].split("\n")
        process_commands(commands, isQuestion)
        for arg in sys.argv:
            if arg.startswith("BetweenEventDelay="):
                delay = arg.split("BetweenEventDelay=")[1]
                print(delay)
