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

import subprocess
import matplotlib.pyplot as plt

concepts = {}
events = {}
cmd = "./NAR shell InspectionOnExit < ./examples/nal/example1.nal"
lines = subprocess.getoutput(cmd).split("\n")
concepts = {}
Active = False

for l in lines:
    L = l.split(":")
    if Active and len(L) >= 2:
        term = L[0].strip()
        information = eval(":".join(L[1:]).strip())
        concepts[term] = information
    elif "*concepts" in l:
        Active = True
    elif "*done" in l:
        break
        
ConceptPriorities=[]
ConceptUseCount=[]
for c in concepts.values():
    ConceptPriorities += [c["priority"]]
    ConceptUseCount += [c["useCount"]]
plt.hist(ConceptPriorities, log=True)
plt.savefig("ConceptPrioritiesHistogram.png")
plt.hist(ConceptUseCount, log=True)
plt.savefig("ConceptUseCountHistogram.png")
print("Average priority: " + str((sum(ConceptPriorities))/float(len(ConceptPriorities))))
print("Average use count: " + str((sum(ConceptUseCount))/float(len(ConceptUseCount))))
