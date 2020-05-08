"""
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
 * """

#NAR NLP Shell (needs nltk)
#Can "parse" English with roughly the following structure to Narsese:  
#...[[[adj] subject] ... [adv] predicate] ... [adj] object ... [prep adj object2] conj

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
def words_and_types(text):
    tokens = [word.lower() for word in word_tokenize(text) if word.isalpha()]
    wordtypes_ordered = nltk.pos_tag(tokens, tagset='universal')
    wordtypes = dict(wordtypes_ordered)
    lemma = WordNetLemmatizer()
    tokens = [lemma.lemmatize(word, pos = wordnet_tag(wordtypes[word])) for word in tokens]
    wordtypes = dict([(tokens[i], wordtypes_ordered[i][1]) for i in range(len(tokens))])
    sys.stdout.flush()
    return tokens, wordtypes

#output the Narsese, replacing question words with question variables
questionwords = set(["what", "where", "which", "when", "who"])
def output(text, replaceQuestionWords=True):
    if replaceQuestionWords:
        for x in questionwords:
            text = text.replace(x, "?1")
    print(text)
    sys.stdout.flush()
    
#return word type for a word, treating question words (who, what etc.) also as nouns
def isWordType(word, wordtype):
    global questionwords
    if word in wordtypes:
        if wordtypes[word] == "PRON" and wordtype == "NOUN":
            questionwords.add(word)
            return True
        return wordtype == wordtypes[word]

while True:
    try:
        sentence = input().strip()
    except:
        break
    if sentence.startswith("*") or sentence.startswith("//") or sentence.isdigit() or sentence.startswith('(') or sentence.startswith('<'):
        output(sentence)
        continue
    output("//Input sentence: " + sentence)
    (words, wordtypes) = words_and_types(sentence + " and")
    punctuation = "?" if "?" in sentence else "."
    output("//Word types: " + str(wordtypes))
    subject = ""
    subject_modifiers = "_subject_"
    predicate = ""
    predicate_modifiers = "_predicate_"
    object = ""
    object_modifiers = "_object_"
    prep = "" #prepositions
    prep_object = ""
    prep_object_modifiers = "_prep_object_"
    lastsubject = ""
    lastpredicate = ""
    for i in range(len(words)):
        word = words[i]
        if prep != "":
            if isWordType(word, 'CONJ'): #we reached the end of the sentence, if there was a preposition, build relation between subject and the preposition verb (in/at etc.) and noun (garden, forest...)
                if subject != "" and prep_object != "":
                    output("<" + subject_modifiers.replace("_subject_", subject) + " --> (" + prep + " /1 " + prep_object_modifiers.replace("_prep_object_", prep_object)+")>" + punctuation + " :|:")
            elif isWordType(word, "NOUN"): #bind the object of the preposition
                prep_object = word
            elif isWordType(word, 'ADJ'): #allow modifying the object of the preposition with adjectives
                prep_object_modifiers = "(& [" + word + "] _prep_object_ )".replace("_prep_object_", prep_object_modifiers)
        if prep == "":
            if isWordType(word, 'NOUN'): #nouns bind subject and object in this order
                if subject == "":
                    subject = word
                elif object == "":
                    object = word
            elif isWordType(word, 'ADJ') or isWordType(word, 'ADV'): #adjectives/adverbs modify the next coming noun/verb
                if predicate == "be" and i+1 < len(words) and isWordType(words[i+1], 'CONJ'): #for is-a sentences where the adjective serves as object
                    output(("<" + subject_modifiers + " --> [" + word + "]>" + punctuation + " :|:").replace("_subject_", subject))
                if subject == "":
                    subject_modifiers = "(& [" + word + "] _subject_ )".replace("_subject_", subject_modifiers)
                elif predicate == "":
                    predicate_modifiers = "(& [" + word + "] _predicate_ )".replace("_predicate_", predicate_modifiers)
                elif object == "":
                    object_modifiers = "(& [" + word + "] _object_ )".replace("_object_", object_modifiers)
            elif isWordType(word, 'VERB'):
                predicate = word
            elif isWordType(word, 'CONJ') or isWordType(word, 'ADP'): #prepositions, conjs
                if isWordType(word, 'CONJ') and object == "": #use the new subject as object, and last subject as the subject, if object was not identified
                    object = subject
                    object_modifiers = subject_modifiers.replace("_subject_","_object_")
                    subject_modifiers = subject = lastsubject
                    if predicate == "": #also use the last predicate if it was not in this sentence "segement"
                        predicate = lastpredicate
                lastsubject = subject_modifiers.replace("_subject_", subject)
                lastpredicate = predicate_modifiers.replace("_predicate_", predicate)
                if subject != "" and predicate !="" and object == "":
                    object = subject
                if subject != "" and predicate != "" and object != "" and not (subject in questionwords and object in questionwords): #output Narsese relation if all pieces are together, with a special case for be/Inheritance
                    if lastpredicate == "be":
                        if subject_modifiers.replace("_subject_", subject) != object_modifiers.replace("_object_", object):
                            output(("<" + subject_modifiers + " --> " + object_modifiers + ">" + punctuation + " :|:").replace("_subject_", subject).replace("_object_", object))
                    else:
                        if object == subject:
                            output(("<" + subject_modifiers + " --> [" + predicate_modifiers + "]>" + punctuation + " :|:").replace("_subject_", subject).replace("_predicate_", predicate))
                        else:
                            output(("<" + subject_modifiers + " --> (" + predicate_modifiers + " /1  " + object_modifiers + ")>" + punctuation + " :|:").replace("_subject_", subject).replace("_predicate_", predicate).replace("_object_", object))
                if isWordType(word, 'ADP'): #identify prepositions in which case we keep the assignments
                    prep = word
                else: #if not we are in a new sentence segement, reset variables
                    subject = ""
                    predicate = ""
                    object = ""
                    subject_modifiers = "_subject_"
                    predicate_modifiers = "_predicate_"
                    object_modifiers = "_object_"
