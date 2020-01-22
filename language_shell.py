#YAN NLP Shell
#can "parse" anything with roughly the structure:  
#...[[[adj] subject] ... [adv] predicate] ... [adj] object ... [prep adj object2] conj

#pip install nltk
import sys
import thread
import time
from subprocess import Popen, PIPE, STDOUT
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
proc = subprocess.Popen(["./YAN","shell"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

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
questionwords = set([])
def output(text):
    for x in questionwords:
        text = text.replace(x, "?1")
    text = text.replace("what","?1").replace("where","?1").replace("which","?1").replace("when","?1").replace("who","?1")
    print(text)
    sys.stdout.flush()
    proc.stdin.write(text + "\n")
    
#return word type for a word, treating question words (who, what etc.) also as nouns
def isWordType(word, wordtype):
    global questionwords
    if word in wordtypes:
        if wordtypes[word] == "PRON" and wordtype == "NOUN":
            questionwords.add(word)
            return True
        return wordtype == wordtypes[word]

def receive_thread(a):
    global cnt
    while True:
        newstr = proc.stdout.readline().strip()
        if newstr != "":
            print(newstr)
            sys.stdout.flush()
       
thread.start_new_thread(receive_thread,(1,))
sys.stdout.flush()
timeout = 0
while True:
    try:
        sentence = raw_input()
    except: #exit but let reasoning finish according to the steps in the file using pessmistic estimation:
        time.sleep(1+timeout/100) #to not exit before: 100 steps per second should be doable! :)
        exit(0) #eof from cat (TODO check in a better way, TODO timeout can be done in a better way as well)
    if sentence.strip().isdigit():
        timeout += int(sentence.strip())
        proc.stdin.write(sentence.strip() + "\n")
        continue
    if sentence.strip().startswith("*") or sentence.strip().startswith("//"):
        proc.stdin.write(sentence.strip() + "\n")
        continue
    (words, wordtypes) = words_and_types(sentence + " and")
    punctuation = "?" if "?" in sentence else "."
    print("Word types: " + str(wordtypes))
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
    if len(words) == 4: #"sam likes tim" special case to circumvent postag issues
        if words[1] == "be":
            output("<" + words[0] + " --> " + words[2] + ">" + punctuation + " :|:")
        else:
            output("<" + words[0] + " --> (" + words[1] + " /1 " + words[2] + ")>" + punctuation + " :|:")
        continue
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
                if subject != "" and predicate != "" and object != "": #output Narsese relation if all pieces are together, with a special case for be/Inheritance
                    if lastpredicate == "be":
                        output(("<" + subject_modifiers + " --> " + object_modifiers + ">" + punctuation + " :|:").replace("_subject_", subject).replace("_object_", object))
                    else:
                        output(("<" + subject_modifiers + " --> ( " + predicate_modifiers + " /1  " + object_modifiers + ")>" + punctuation + " :|:").replace("_subject_", subject).replace("_predicate_", predicate).replace("_object_", object))
                if isWordType(word, 'ADP'): #identify prepositions in which case we keep the assignments
                    prep = word
                else: #if not we are in a new sentence segement, reset variables
                    subject = ""
                    predicate = ""
                    object = ""
                    subject_modifiers = "_subject_"
                    predicate_modifiers = "_predicate_"
                    object_modifiers = "_object_"
            else:
                print("unknown word type: " + word)
                sys.stdout.flush()
