from math import exp
import sys
sys.path.append('../Python/')
import NAR
import random
import json

#Author: Robert Johansson

conditions = [["<{light} --> [on]>. :|:", "^clap"],
              ["<{light} --> [off]>. :|:", "^wave"]]

NAR.AddInput("*babblingops=2")
NAR.AddInput("*motorbabbling=0.9")
NAR.AddInput("*setopname 1 ^clap")
NAR.AddInput("*setopname 2 ^wave")

NAR.AddInput("*volume=0")


print("Baseline")
print("********")

for _ in range(8):
    n = random.randint(0, 1)
    setup = conditions[n]
    input = conditions[n][0]
    expected_op = conditions[n][1]

    NAR.AddInput(input, Print=False)
    response = NAR.AddInput("G! :|:", Print=False)
    response.pop("raw")
    t = response["input"][0]["occurrenceTime"]
    op = response["executions"][0]["operator"]
    response.pop("input")

    if response["reason"]:
        desire = response["reason"]["desire"]
        term = response["reason"]["hypothesis"]["term"]
        f = response["reason"]["hypothesis"]["truth"]["frequency"]
        c = response["reason"]["hypothesis"]["truth"]["confidence"]
    else:
        desire = ""
        term = ""
        f = ""
        c = ""

    print("Time: " + t)
    print("Operator: " + op)
    print("Desire: " + desire)
    print("Term: " + term)
    print("Frequency: " + f)
    print("Confidence: " + c)
    print()
    #print(h)
    #print(response)
    #print(json.dumps(response, indent = 4))


    # if op == expected_op:
    #     NAR.AddInput("G. :|:", Print=False)
    # elif op != expected_op:
    #     NAR.AddInput("(! G). :|:", Print=False)
    NAR.AddInput("100", Print=False)


print("")
print("")
print("With reinforcer")
print("********")
print("")
print("")


for _ in range(16):
    n = random.randint(0, 1)
    setup = conditions[n]
    input = conditions[n][0]
    expected_op = conditions[n][1]

    NAR.AddInput(input, Print=False)
    response = NAR.AddInput("G! :|:", Print=False)
    response.pop("raw")
    t = response["input"][0]["occurrenceTime"]
    op = response["executions"][0]["operator"]
    response.pop("input")

    if response["reason"]:
        desire = response["reason"]["desire"]
        term = response["reason"]["hypothesis"]["term"]
        f = response["reason"]["hypothesis"]["truth"]["frequency"]
        c = response["reason"]["hypothesis"]["truth"]["confidence"]
    else:
        desire = ""
        term = ""
        f = ""
        c = ""

    print("Time: " + t)
    print("Operator: " + op)
    print("Desire: " + desire)
    print("Term: " + term)
    print("Frequency: " + f)
    print("Confidence: " + c)
    print()
    #print(h)
    #print(response)
    #print(json.dumps(response, indent = 4))


    if op == expected_op:
        NAR.AddInput("G. :|:", Print=False)
    elif op != expected_op:
        NAR.AddInput("G. :|: {0.0 0.9}", Print=False)
    NAR.AddInput("100", Print=False)



foo = {'reason': {'desire': '0.569243',
                  'hypothesis': {'occurrenceTime': 'eternal', 'punctuation': '.',
                                 'term': '<(<{light} --> [off]> &/ ^wave) =/> G>', 'truth': {'frequency': '1.000000', 'confidence': '0.213712'}},
                  'precondition': {'occurrenceTime': '209', 'punctuation': '.',
                                   'term': '<{light} --> [off]>', 'truth': {'frequency': '1.000000', 'confidence': '0.900000'}}}}

bar = {'input': [{'occurrenceTime': '4', 'punctuation': '.', 'term': '(! G)', 'truth': {'frequency': '1.000000,', 'confidence': '0.900000'}}],
       'derivations': [{'occurrenceTime': 'eternal', 'punctuation': '.', 'term': 'dt=2.000000 <(<{light} --> [on]> &/ ^wave) =/> (! G)>',
                        'truth': {'frequency': '1.000000,', 'confidence': '0.213712'}},
                       {'occurrenceTime': 'eternal', 'punctuation': '.', 'term': 'dt=3.000000 <<{light} --> [on]> =/> (! G)>',
                       'truth': {'frequency': '1.000000,', 'confidence': '0.226692'}},
                       {'occurrenceTime': '4', 'punctuation': '.', 'term': 'G',
                       'truth': {'frequency': '0.000000,', 'confidence': '0.900000'}},
                       {'occurrenceTime': '4', 'punctuation': '.', 'term': 'G', 'truth': {'frequency': '0.000000,', 'confidence': '0.900000'}}],
       'answers': [],
       'executions': [],
       'reason': None,
       'raw': 'Input: (! G). :|: occurrenceTime=4 Priority=1.000000 Truth: frequency=1.000000, confidence=0.900000\nDerived: dt=2.000000 <(<{light} --> [on]> &/ ^wave) =/> (! G)>. Priority=0.159391 Truth: frequency=1.000000, confidence=0.213712\nDerived: dt=3.000000 <<{light} --> [on]> =/> (! G)>. Priority=0.171089 Truth: frequency=1.000000, confidence=0.226692\nDerived: G. :|: occurrenceTime=4 Priority=0.050000 Truth: frequency=0.000000, confidence=0.900000\nDerived: G. :|: occurrenceTime=4 Priority=0.050000 Truth: frequency=0.000000, confidence=0.900000'}

baz = {'input': [{'occurrenceTime': '210', 'punctuation': '!', 'term': 'G', 'truth': {'frequency': '1.000000,', 'confidence': '0.900000'}},
                 {'occurrenceTime': '210', 'punctuation': '.', 'term': '^wave', 'truth': {'frequency': '1.000000,', 'confidence': '0.900000'}}],
       'derivations': [],
       'answers': [],
       'executions': [{'operator': '^wave', 'arguments': []}],
       'reason': {'desire': '0.569243',
                  'hypothesis': {'occurrenceTime': 'eternal', 'punctuation': '.', 'term': '<(<{light} --> [off]> &/ ^wave) =/> G>',
                                 'truth': {'frequency': '1.000000', 'confidence': '0.213712'}},
                  'precondition': {'occurrenceTime': '209', 'punctuation': '.', 'term': '<{light} --> [off]>',
                                   'truth': {'frequency': '1.000000', 'confidence': '0.900000'}}},
       'raw': 'Input: G! :|: occurrenceTime=210 Priority=1.000000 Truth: frequency=1.000000, confidence=0.900000\ndecision expectation=0.569243 implication: <(<{light} --> [off]> &/ ^wave) =/> G>. Truth: frequency=1.000000 confidence=0.213712 dt=2.000000 precondition: <{light} --> [off]>. :|: Truth: frequency=1.000000 confidence=0.900000 occurrenceTime=209\n^wave executed with args\nInput: ^wave. :|: occurrenceTime=210 Priority=1.000000 Truth: frequency=1.000000, confidence=0.900000'}
