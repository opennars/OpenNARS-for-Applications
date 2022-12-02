import sys
from os.path import exists
import pickle 
fname = "mem.pkl"

key, index = (sys.argv[1], sys.argv[2])
mem = {}
if exists(fname):
    with open(fname, 'rb') as f:
        mem = pickle.load(f)

#return whether key exists
if index == "exists":
    if key in mem:
        if "print" in sys.argv:
            print("//RESTORE: ", str(sys.argv), mem[key])
        exit(0)
    exit(1)

#else return value at index of specified ke
exit(mem[key][int(index)])
