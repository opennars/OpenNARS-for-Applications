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
from subprocess import PIPE, run
import subprocess
import glob

Branch1 = "master"
Branch2 = "QLearningComparison"

#NAR C tests & metrics, only print fully output on failure, always print the metrics:
def ctests(Branch, Example, Args):
    Filename = Example + "_" + Branch + ".txt"
    result = run(Args.split(" "), stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if result.returncode != 0:
        print(result.stdout, result.stderr)
        exit(result.returncode)
    with open(Filename, 'w') as f:
        for line in reversed(result.stdout.split("\n")):
            if "ratio=" in line:
                print(line)
                f.write(line + "\n")
    print("\n" + Example + " successful!")

steps = "5000"
ctests(Branch1, "Pong", "./"+Branch1+"/OpenNARS-for-Applications/NAR pong "+steps)
ctests(Branch2, "Pong", "./"+Branch2+"/OpenNARS-for-Applications/NAR pong "+steps)
ctests(Branch1, "Pong2", "./"+Branch1+"/OpenNARS-for-Applications/NAR pong2 "+steps)
ctests(Branch2, "Pong2", "./"+Branch2+"/OpenNARS-for-Applications/NAR pong2 "+steps)
ctests(Branch1, "Alien", "./"+Branch1+"/OpenNARS-for-Applications/NAR alien "+steps)
ctests(Branch2, "Alien", "./"+Branch2+"/OpenNARS-for-Applications/NAR alien "+steps)
ctests(Branch1, "Cartpole", "./"+Branch1+"/OpenNARS-for-Applications/NAR cartpole "+steps)
ctests(Branch2, "Cartpole", "./"+Branch2+"/OpenNARS-for-Applications/NAR cartpole "+steps)
