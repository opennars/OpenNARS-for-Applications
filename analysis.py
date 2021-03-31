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

import matplotlib.pyplot as plt
import sys

concepts = {}
for l in sys.stdin:
    if ": {" not in l or not l.startswith("//"):
        continue
    L = l.split(":")
    term = L[0].strip()
    information = eval(":".join(L[1:]).strip())
    concepts[term] = information

ConceptPriorities=[]
ConceptUseCount=[]
ConceptLastUsedDistance=[]
LastLastUsed = 0
for c in concepts.values():
    ConceptPriorities += [c["priority"]]
    ConceptUseCount += [c["useCount"]]
    ConceptLastUsedDistance += [c["lastUsed"]]
    LastLastUsed = max(LastLastUsed, c["lastUsed"])
for i in range(len(ConceptLastUsedDistance)):
    ConceptLastUsedDistance[i] = LastLastUsed - ConceptLastUsedDistance[i]
plt.figure()
plt.title("ConceptPriorities histogram")
plt.xlabel("ConceptPriority")
plt.ylabel("Count")
plt.hist(ConceptPriorities, log=True)
plt.savefig("ConceptPrioritiesHistogram.png")
plt.figure()
plt.title("ConceptUseCount histogram")
plt.xlabel("ConceptUseCount")
plt.ylabel("Count")
plt.hist(ConceptUseCount, log=True)
plt.savefig("ConceptUseCountHistogram.png")
plt.figure()
plt.title("ConceptLastUsedDistance histogram")
plt.xlabel("ConceptLastUsedDistance")
plt.ylabel("Count")
plt.hist(ConceptLastUsedDistance, log=True)
plt.savefig("ConceptLastUsedDistance histogram.png")
print("Average priority: " + str((sum(ConceptPriorities))/float(len(ConceptPriorities))))
print("Average use count: " + str((sum(ConceptUseCount))/float(len(ConceptUseCount))))
