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

import sys
import pickle
import subprocess
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

if len(sys.argv) == 3:
    branches = [sys.argv[1], sys.argv[2]]
else:
    with open('branches', 'rb') as fp:
        branches = pickle.load(fp)
with open('seeds', 'rb') as fp:
    seeds = pickle.load(fp)
with open('examples', 'rb') as fp:
    examples = pickle.load(fp)
with open('successCriterias', 'rb') as fp:
    successCriterias = pickle.load(fp)

def Plot(example, successCriteria):
    plt.figure()
    plt.title(example)
    plt.ylabel("Success ratio")
    plt.xlabel("Time")
    colors = ['g', 'y', 'r', 'b']
    p1 = mpatches.Patch(color = "green", label = branches[0])
    p2 = mpatches.Patch(color = "yellow", label = branches[1])
    p3 = mpatches.Patch(color = "red", label = branches[0] + " avg.")
    p4 = mpatches.Patch(color = "blue", label = branches[1] + " avg.")
    plt.legend(handles=[p1, p2, p3, p4])
    BranchRatios = {}
    k=-1
    for Branch in branches:
        k+=1
        for seed in seeds:
            filename = example + "_" + Branch + "_" + str(seed)
            with open(filename + ".txt", 'r') as f:
                lines = f.readlines()
            lines = reversed(lines)
            ratios = []
            t = 0
            for l in lines:
                if l.strip() == "":
                    continue
                t += 1
                ratio = float(l.split(successCriteria+"=")[1].split(" ")[0].split(",")[0].replace("-nan","0").replace("nan","0"))
                ratios.append(ratio)
                if Branch not in BranchRatios:
                    BranchRatios[Branch] = {}
                if t not in BranchRatios[Branch]:
                    BranchRatios[Branch][t] = []
                BranchRatios[Branch][t].append(ratio)
            plt.plot(ratios, colors[k])
    for Branch in branches:
        k += 1
        BranchRatioAvgs = []
        for t in range(1, 1+len(BranchRatios[Branch])):
            BranchRatioAvg = sum(BranchRatios[Branch][t]) / float(len(BranchRatios[Branch][t]))
            BranchRatioAvgs.append(BranchRatioAvg)
        plt.plot(BranchRatioAvgs, colors[k])
    plt.savefig(example + ".png")

for i in range(len(examples)):
    Plot(examples[i], successCriterias[i])
