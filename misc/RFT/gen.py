from math import exp
import sys
sys.path.append('../Python/')
import NAR
import random
import json

#Author: Robert Johansson

# <{sample} --> [A1]>. :|:
# <{left} --> [B1]>. :|:
# <{right} --> [B2]>. :|:
# ^left

# <{sample} --> [A1]>. :|:
# <{left} --> [B2]>. :|:
# <{right} --> [B1]>. :|:
# ^right

# <{sample} --> [A2]>. :|:
# <{left} --> [B1]>. :|:
# <{right} --> [B2]>. :|:
# ^right

# <{sample} --> [A2]>. :|:
# <{left} --> [B2]>. :|:
# <{right} --> [B1]>. :|:
# ^left

NAR.AddInput("*babblingops=2")
NAR.AddInput("*motorbabbling=0.9")
NAR.AddInput("*setopname 1 ^left")
NAR.AddInput("*setopname 2 ^right")

conditions = [["<{sample} --> [A1]>. :|:", "<{left} --> [A1]>. :|:", "<{right} --> [A2]>. :|:", "^left"],
              ["<{sample} --> [A1]>. :|:", "<{left} --> [A2]>. :|:",
                  "<{right} --> [A1]>. :|:", "^right"],
              ["<{sample} --> [A2]>. :|:", "<{left} --> [A1]>. :|:",
                  "<{right} --> [A2]>. :|:", "^right"],
              ["<{sample} --> [A2]>. :|:", "<{left} --> [A2]>. :|:", "<{right} --> [A1]>. :|:", "^left"]]

test_conditions = [["<{sample} --> [A3]>. :|:", "<{left} --> [A3]>. :|:", "<{right} --> [A4]>. :|:", "^left"],
                   ["<{sample} --> [A3]>. :|:", "<{left} --> [A4]>. :|:",
                    "<{right} --> [A3]>. :|:", "^right"],
                   ["<{sample} --> [A4]>. :|:", "<{left} --> [A3]>. :|:",
                    "<{right} --> [A4]>. :|:", "^right"],
                   ["<{sample} --> [A4]>. :|:", "<{left} --> [A4]>. :|:", "<{right} --> [A3]>. :|:", "^left"]]


NAR.AddInput("*volume=0")

# seq = [0, 1]
# print("Learning A1->B1")
# print("***************")


# for _ in range(4):
#     s = random.sample(seq, 2)

#     for i in s:
#         trial = conditions[i]
#         sample = trial[0]
#         left = trial[1]
#         right = trial[2]
#         expected_op = trial[3]

#         NAR.AddInput(sample, Print=True)
#         NAR.AddInput(left, Print=True)
#         NAR.AddInput(right, Print=True)

#         response = NAR.AddInput("G! :|:", Print=True)

#         op = response["executions"][0]["operator"]

#         if op == expected_op:
#             print("\x1B[31mCORRECT: " + op + "\x1B[0m")
#             NAR.AddInput("G. :|:", Print=True)
#         elif op != expected_op:
#             print("INCORRECT: " + op)
#             NAR.AddInput("G. :|: {0.0 0.9}", Print=True)

#         NAR.AddInput("100", Print=True)

#     print("End of block")
#     print()


print()
print()
print()
print()
print()
print()


# seq = [2, 3]
# print("Learning A2->B2")
# print("***************")

# for _ in range(4):
#     s = random.sample(seq, 2)

#     for i in s:
#         trial = conditions[i]
#         sample = trial[0]
#         left = trial[1]
#         right = trial[2]
#         expected_op = trial[3]

#         NAR.AddInput(sample, Print=True)
#         NAR.AddInput(left, Print=True)
#         NAR.AddInput(right, Print=True)

#         response = NAR.AddInput("G! :|:", Print=True)

#         op = response["executions"][0]["operator"]

#         if op == expected_op:
#             print("\x1B[31mCORRECT: " + op + "\x1B[0m")
#             NAR.AddInput("G. :|:", Print=True)
#         elif op != expected_op:
#             print("INCORRECT: " + op)
#             NAR.AddInput("G. :|: {0.0 0.9}", Print=True)

#         NAR.AddInput("100", Print=True)

#     print("End of block")
#     print()


seq = [0, 1, 2, 3]
print("Learning A1->A1 and A2->A2")
print("***************")
print()

for _ in range(8):
    s = random.sample(seq, 4)

    for i in s:
        trial = conditions[i]
        sample = trial[0]
        left = trial[1]
        right = trial[2]
        expected_op = trial[3]

        NAR.AddInput(sample, Print=True)
        NAR.AddInput(left, Print=True)
        NAR.AddInput(right, Print=True)

        response = NAR.AddInput("G! :|:", Print=True)

        op = response["executions"][0]["operator"]

        if op == expected_op:
            print("\x1B[31mCORRECT: " + op + "\x1B[0m")
            NAR.AddInput("G. :|:", Print=True)
        elif op != expected_op:
            print("INCORRECT: " + op)
            NAR.AddInput("G. :|: {0.0 0.9}", Print=True)

        NAR.AddInput("100", Print=True)

    print("End of block")
    print()


seq = [0, 1, 2, 3]
print("TESTING A1->A1 and A2->A2")
print("***************")
print()

for _ in range(4):
    s = random.sample(seq, 4)

    for i in s:
        trial = conditions[i]
        sample = trial[0]
        left = trial[1]
        right = trial[2]
        expected_op = trial[3]

        NAR.AddInput(sample, Print=True)
        NAR.AddInput(left, Print=True)
        NAR.AddInput(right, Print=True)

        response = NAR.AddInput("G! :|:", Print=True)

        op = response["executions"][0]["operator"]

        if op == expected_op:
            print("\x1B[31mCORRECT: " + op + "\x1B[0m")
        elif op != expected_op:
            print("INCORRECT: " + op)

        NAR.AddInput("100", Print=True)

    print("End of block")
    print()


seq = [0, 1, 2, 3]
print("TESTING A3->A3 and A4->A4")
print("***************")
print()

for _ in range(4):
    s = random.sample(seq, 4)

    for i in s:
        trial = test_conditions[i]
        sample = trial[0]
        left = trial[1]
        right = trial[2]
        expected_op = trial[3]

        NAR.AddInput(sample, Print=True)
        NAR.AddInput(left, Print=True)
        NAR.AddInput(right, Print=True)

        response = NAR.AddInput("G! :|:", Print=True)

        op = response["executions"][0]["operator"]

        if op == expected_op:
            print("\x1B[31mCORRECT: " + op + "\x1B[0m")
        elif op != expected_op:
            print("INCORRECT: " + op)

        NAR.AddInput("100", Print=True)

    print("End of block")
    print()

    # response.pop("raw")
    # t = response["input"][0]["occurrenceTime"]
    # response.pop("input")

    # if response["reason"]:
    #     desire = response["reason"]["desire"]
    #     term = response["reason"]["hypothesis"]["term"]
    #     f = response["reason"]["hypothesis"]["truth"]["frequency"]
    #     c = response["reason"]["hypothesis"]["truth"]["confidence"]
    # else:
    #     desire = ""
    #     term = ""
    #     f = ""
    #     c = ""

    # print("Time: " + t)
    # print("Operator: " + op)
    # print("Desire: " + desire)
    # print("Term: " + term)
    # print("Frequency: " + f)
    # print("Confidence: " + c)
    # print()
    # print(h)
    # print(response)
    #print(json.dumps(response, indent = 4))


print("Finished!")

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
