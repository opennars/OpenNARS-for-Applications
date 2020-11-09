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
import matplotlib.patches as mpatches

def Plot(Example):
    plt.figure()
    plt.title(Example)
    plt.ylabel("Success ratio")
    plt.xlabel("Time")
    colors = ['r', 'g']
    red_patch = mpatches.Patch(color = "red", label = "ONA")
    green_patch = mpatches.Patch(color = "green", label = "QL")
    plt.legend(handles=[red_patch, green_patch])
    k=-1
    for Branch in ["master", "QLearningComparison"]:
        k+=1
        Filename = Example + "_" + Branch
        with open(Filename+".txt", 'r') as f:
            lines = f.readlines()
        lines = reversed(lines)
        ratios = []
        for l in lines:
            if l.strip() == "":
                continue
            ratios.append(float(l.split("ratio=")[1].split(" ")[0].split(",")[0].replace("-nan","0").replace("nan","0")))
        plt.plot(ratios[0:5000], colors[k])
    plt.savefig(Filename + ".png")

for example in ["Pong", "Pong2", "Alien", "Cartpole"]:
    Plot(example)
