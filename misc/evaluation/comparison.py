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

from copy import deepcopy
from pathlib import Path
from subprocess import PIPE, run
import os
import sys
import subprocess
import glob
import pickle
import random

try:
    branches = list(set(sys.argv[1:])-set(["SkipFolderSetup"]))
except:
    print("Usage: python3 comparison.py branchName1 [branchName2] ... [branchNameN]")
    exit(0)
SkipFolderSetup = "SkipFolderSetup" in sys.argv
if not SkipFolderSetup:
    os.system("rm -rf OpenNARS-for-Applications")
    os.system("git clone https://github.com/opennars/OpenNARS-for-Applications")
    for b in branches:
        scriptroot = os.getcwd()
        os.system("rm -rf " + b)
        os.system("mkdir " + b)
        os.system("cp -r OpenNARS-for-Applications ./" + b + "/")
        os.chdir("./" + b + "/OpenNARS-for-Applications/")
        os.system("git checkout " + b)
        os.chdir(scriptroot)
examples = ["pong", "pong2", "alien", "cartpole", "robot", "minigrid_ona.py"]
steps = [10000, 10000, 10000, 10000, 1500, 30000]
seeds = [42, 1337, 666, 900, 10000, 77, 2324, 22, 11, 876]
successCriterias = ["ratio", "ratio", "ratio", "ratio", "eaten", "successes"]
#alternatively, but it won't produce reproducible plots:
#seeds=[random.randint(0,100000) for i in range(10)]

#NAR C tests & metrics, only print fully output on failure, always print the metrics:
def ctests(branch, example, steps, seed, successCriteria):
    folder = "./" + branch + "/OpenNARS-for-Applications/"
    print("sh " + folder + "build.sh -DSEED="+str(seed))
    os.system("cd " + folder)
    basePath = Path.cwd()
    os.chdir(folder)
    os.system("sh ./build.sh -DSEED="+str(seed))
    os.chdir(basePath)
    binAndArgs = folder + "NAR " + example + " " + str(steps)
    filename = example + "_" + branch + "_" + str(seed) + ".txt"
    result = run(binAndArgs.split(" "), stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if result.returncode != 0:
        print(result.stdout, result.stderr)
        exit(result.returncode)
    with open(filename, 'w') as f:
        for line in reversed(result.stdout.split("\n")):
            if successCriteria+"=" in line:
                print(line)
                f.write(line + "\n")
    print("\n" + example + " successful!")

#NAR Py tests & metrics, only print fully output on failure, always print the metrics:
def pytests(branch, example, steps, seed, successCriteria):
    folder = "./" + branch + "/OpenNARS-for-Applications/"
    print("sh " + folder + "build.sh -DSEED="+str(seed))
    os.system("cd " + folder)
    basePath = Path.cwd()
    os.chdir(folder)
    os.system("sh ./build.sh -DSEED="+str(seed))
    os.chdir("./misc/Python/")
    binAndArgs = "python3 " + example + " " + str(steps)
    filename = example + "_" + branch + "_" + str(seed) + ".txt"
    result = run(binAndArgs.split(" "), stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if result.returncode != 0:
        print(result.stdout, result.stderr)
        exit(result.returncode)
    os.chdir(basePath)
    with open(filename, 'w') as f:
        for line in reversed(result.stdout.split("\n")):
            if successCriteria+"=" in line:
                print(line)
                f.write(line + "\n")
    print("\n" + example + " successful!")
    

with open('branches', 'wb') as fp:
    pickle.dump(branches, fp)
with open('seeds', 'wb') as fp:
    pickle.dump(seeds, fp)
with open('successCriterias', 'wb') as fp:
    pickle.dump(successCriterias, fp)
with open('examples', 'wb') as fp:
    pickle.dump(examples, fp)
for i in range(len(examples)):
    example = examples[i]
    for seed in seeds:
        for branch in branches:
            if example.endswith(".py"):
                pytests(branch, example, steps[i], str(seed), successCriterias[i])
            else:
                ctests(branch, example, steps[i], str(seed), successCriterias[i])

