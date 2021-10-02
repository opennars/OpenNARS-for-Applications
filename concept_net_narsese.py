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
 
# >ConceptNet knowledge channel which queries for additional knowledge
#  Usage: python3 concept_net_narsese.py [maxConceptNetQueries] [queryOnBeliefs] [allRelations]
#  whereby maxConceptNetQueries indicates how many query results per
#  relationship are utilized for each atomic term in the input Narsese
#  and QueryOnbeliefs allows to make queries for belief input too

import re
import requests
import codecs
import sys
import nltk
from nltk.corpus import brown
from collections import Counter
sys.stdout = codecs.getwriter("utf-8")(sys.stdout.detach())

nltk.download('brown')
wordcounts = Counter(brown.words())

def toNarsese(subject_relation_predicate):
    (subject, relation, predicate) = subject_relation_predicate
    if relation == "IsA":
        if subject[0].isupper():
            return "<{" + subject + "} --> " + predicate + ">."
        else:
            return "<" + subject + " --> " + predicate + ">."
    if relation == "InstanceOf":
        if subject[0].islower():
            return "<" + subject + " --> " + predicate + ">."
        else:
            return "<{" + subject + "} --> " + predicate + ">."
    if relation == "HasProperty":
        return "<" + subject + " --> [" + predicate + "]>."
    if relation == "DistinctFrom":
        return "(--,<" + subject + " <-> " + predicate + ">)."
    if relation == "SimilarTo":
        return "<" + subject + " <-> " + predicate + ">."
    if relation == "Causes":
        return "<" + subject + " =/> " + predicate + ">."
    return "<(" + subject + " * " + predicate + ") --> " + relation.replace("PartOf", "part_of").replace("HasA", "have").replace("MadeOf", "make_of").replace("Desires", "want").lower() + ">."

def unwrap(rel):
    parts = rel.split("[")[1].split("]")[0].replace("/c/en/", "").replace("/n/", "").replace("/r/", "").replace("/", "").split(",");
    return (parts[1], parts[0], parts[2])

def queryConceptNet(maxAmount, term, side, relation):
    ret = []
    req = requests.get("http://api.conceptnet.io/query?" + side + "=/c/en/" + term + "&rel=/r/" + relation + "&limit=" + str(maxAmount))
    edges = req.json()["edges"]
    for edge in edges:
        (s,v,p) = unwrap(edge["@id"])
        if (s == term or p == term) and "_" not in s and "_" not in p:
            count = wordcounts[p] if s == term else wordcounts[s]
            ret.append((toNarsese((s,v,p)),count))
    return ret

def queryMeaning(term, maxAmount, selectAmount, isEvent, querySpecificQuestion, question):
    ret = []
    Relations = ["IsA", "InstanceOf", "HasProperty", "SimilarTo"] + ["DistinctFrom", "PartOf", "HasA", "MadeOf", "Causes", "Desires"]
    if querySpecificQuestion:
        if "} --> [" in question:
            Relations = ["HasProperty"]
        elif " --> [" in question:
            Relations = ["HasProperty"]
        elif "} --> " in question:
            Relations = ["InstanceOf"]
        elif " <-> " in question:
            Relations = ["SimilarTo"]
        elif "!" in question and " --> " in question:
            Relations = ["DistinctFrom"]
        elif " --> part_of" in question:
            Relations = ["PartOf"]
        elif " --> have" in question:
            Relations = ["HasA"]
        elif " --> make_of" in question:
            Relations = ["MadeOf"]
        elif " --> want" in question:
            Relations = ["Desires"]
        elif " =/> "  in question:
            Relations = ["causes"]
        elif " --> " in question:
            Relations = ["IsA"]
    for rel in Relations:
        for side in ["end", "start"]: #extension and intenstion query
            ret.extend(queryConceptNet(maxAmount, term, side, rel))
    ret.sort(key = lambda T: -T[1])
    if not querySpecificQuestion:
        ret = ret[:selectAmount]
    selected = 0
    returnlist = []
    for T in ret:
        queryPart = re.escape(question.replace(" ","")).replace("\?1","([a-zA-Z0-9]|_)*")
        pattern = T[0].replace(" ", "")
        #print("//MATCH ATTEMPT " + queryPart + " " + pattern + str(bool(re.search(queryPart, pattern))))
        if not querySpecificQuestion or re.search(queryPart, pattern):
            returnlist += [T[0] + (" :|:" if isEvent else "")]
            selected +=1
            if selected >= selectAmount:
                break
    return returnlist

def extractAtomicTerms(inp):
    L = []
    atomicTerm = ""
    for x in inp:
        if x >= "a" and x <= "z" or x >= "A" and x <= "Z" or x >= "0" and x <= "9":
            atomicTerm += x
        else:
            if len(atomicTerm) > 0:
                L += [atomicTerm]
                atomicTerm = ""
    return L

maxAmount = 5
selectAmount = 5
for arg in sys.argv:
    if arg.startswith("maxAmount="):
        maxAmount = int(arg.split("maxAmount=")[1])
    elif arg.startswith("selectAmount="):
        maxAmount = int(arg.split("selectAmount=")[1])
maxAmount = 5 if len(sys.argv) <= 1 else int(sys.argv[1]) #per relation
selectAmount = 1 if len(sys.argv) <= 2 else int(sys.argv[2]) #in total
queryOnBeliefs = "queryOnBeliefs=true" in sys.argv
queryOnQuestions = "queryOnQuestions=false" not in sys.argv
querySpecificQuestion = "querySpecificQuestion=false" not in sys.argv
if querySpecificQuestion:
    maxAmount = max(maxAmount, 30) #at least 30 results are fine to make sure the specifically asked relation will be included
while True:
    try:
        line = input()
    except:
        exit(0)
    isCommand = line.startswith("*") or line.startswith("//") or line.isdigit()
    isNarsese = line.startswith('(') or line.startswith('<')
    isQuestion = line.strip().endswith("? :|:") or line.strip().endswith("?")
    isEvent = " :|:" in line
    if line.startswith("*queryOnBeliefs=true"):
        queryOnBeliefs = True
    elif line.startswith("*queryOnBeliefs=false"):
        queryOnBeliefs = False
    elif line.startswith("*queryOnQuestions=true"):
        queryOnQuestions = True
    elif line.startswith("*queryOnQuestions=false"):
        queryOnQuestions = False
    elif line.startswith("*querySpecificQuestion=true"):
        querySpecificQuestion = True
    elif line.startswith("*querySpecificQuestion=false"):
        querySpecificQuestion = False
    elif line.startswith("*maxConceptNetQueries="):
        maxAmount = int(line.strip().split("*maxConceptNetQueries=")[1])
        continue
    if isCommand:
        print(line)
        sys.stdout.flush()
        continue
    if isNarsese and ((queryOnQuestions and isQuestion) or (queryOnBeliefs and not isQuestion)):
        atoms = extractAtomicTerms(line)
        cnt = 0
        queryResults = set([])
        for atom in atoms:
            question = line.split(">.")[0].split(").")[0].split(")?")[0].split(">?")[0]
            print(("//Querying concept " + atom + " for relationships") if not querySpecificQuestion else ("//Querying concept " + atom + " for relationship " + question + ">"))
            returnlist = queryMeaning(atom, maxAmount, selectAmount, isEvent, querySpecificQuestion, question)
            for x in returnlist:
                queryResults.add(x)
            cnt += 1
            if cnt >= 2:
                break
        for x in queryResults:
            print(x)
        sys.stdout.flush()
        print("//Querying complete")
    if isNarsese:
        print(line)
