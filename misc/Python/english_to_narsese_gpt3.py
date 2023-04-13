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
from nltk import WordNetLemmatizer
from nltk.corpus import wordnet
import openai

openai.api_key = "YOUR_KEY"
eternal = True #whether to use event or eternal output
gpt_prompt = """
def Relation(noun,verb,noun): ... #put relation into database
def Relation(noun,"IsA",noun): ... #put category into database
def Property(noun,adjective): ... #put property into database
Capture the complete sentence meaning with code that calls the two functions, and only use a single word per argument.
The sentence: 
"""

if "EventOutput" in sys.argv:
    eternal = False

lemma = WordNetLemmatizer()
def Lemmatize(word, tag):
    ret = lemma.lemmatize(word, pos = tag).strip().lower().replace(" ","_")
    questionwords = ["what", "where","which", "who", "when", "something","someone", "somewhere", "somewhen"]
    for word in questionwords:
        ret = ret.replace(word, "?1")
    return ret

def Relation(s, v, p, punctuation_tv):
    s = Lemmatize(s, wordnet.NOUN)
    v = Lemmatize(v, wordnet.VERB)
    p = Lemmatize(p, wordnet.NOUN)
    if v == "isa":
        print(f"<{s} --> {p}>{punctuation_tv}")
    else:
        print(f"<({s} * {p}) --> {v}>{punctuation_tv}")

def Property(s, p, punctuation_tv):
    s = Lemmatize(s, wordnet.NOUN)
    p = Lemmatize(p, wordnet.ADJ)
    print(f"<{s} --> [{p}]>{punctuation_tv}")

def process_commands(commands, isQuestion):
    for x in commands:
        if (x.startswith("Property(") or x.startswith("Relation(")) and x.endswith(")"):
            s_v_p = x.split("(")[1].split(")")[0].replace("\"","").replace("'","").split(",")
            eventMarker = "" if eternal else " :|:"
            punctuation_tv = f"?{eventMarker}" if isQuestion else f".{eventMarker} {{1.0 0.9}}"
            if x.startswith("Property"):
                Property(*s_v_p, punctuation_tv)
            if x.startswith("Relation"):
                Relation(*s_v_p, punctuation_tv)

while True:
    try:
        inp = input().rstrip("\n")
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
