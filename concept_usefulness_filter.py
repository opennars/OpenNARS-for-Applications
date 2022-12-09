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

lines = []
curline = None
started = False
for line in sys.stdin:
    if "//*concepts" in line:
        started = True
        lines = []
    if not started:
        continue
    if line.startswith("*"):
        print(line, end='')
        continue
    if line.startswith("//"):
        if curline != "" and curline != None:
            lines.append(curline)
        curline = ""
        if line.startswith("//*done"):
            started = False
    if curline != None and started:
        curline += line.replace("//", "")
#exit(0)
lines = [line for line in lines if line.strip() != "" and ": { \"priority\":" in line]
lines.sort(key=lambda line: -float(line.split("\"usefulness\": ")[1].split(",")[0]))
displayN = len(lines) if len(sys.argv) <= 1 else int(sys.argv[1])
for (i,x) in enumerate(lines[0:displayN]):
    print("//{i="+str(i)+"}", x, end='', flush=True)
