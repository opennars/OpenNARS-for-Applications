import sys
import os
sys.path.insert(1, os.getcwd()+'/misc/Python')
os.chdir('./misc/Python/')
import NAR
NAR.AddInput("<a --> b>.")
NAR.AddInput("<b --> c>.")
print(NAR.AddInput("*stats"))
NAR.Exit()
