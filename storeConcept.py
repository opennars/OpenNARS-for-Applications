import sys
from os.path import exists
import pickle 
fname = "mem.pkl"

key, frequency, confidence, useCount = (sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
mem = {}
if exists(fname):
    with open(fname, 'rb') as f:
        mem = pickle.load(f)

#add values of specified key only if the result is more confident
if key not in mem or (key in mem and confidence > mem[key][1]): 
    mem[key] = (frequency, confidence, useCount)
    if "print" in sys.argv:
        print("//STORE: ", str(sys.argv), mem[key])
#and save updated dict
with open(fname, 'wb') as f:
    pickle.dump(mem, f)


