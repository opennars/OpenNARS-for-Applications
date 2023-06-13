"""
 * The MIT License
 *
 * Copyright 2023 The OpenNARS authors.
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
 
from Nalifier import *
import NAR

nalifier = Nalifier(1)
NAR.AddInput("*volume=100")

def ProcessNARret(ret):
    if "derivations" in ret and ret["derivations"]:
        for derivation in ret["derivations"]:
            if derivation["punctuation"] == "!":
                if " --> " in derivation["term"] and "[" in derivation["term"]:
                    prop = derivation["term"].split(" --> ")[1].split("[")[1].split("]")[0]
                    nalifier.ShellInput("*PROPERTY_OF_INTEREST=" + prop)

if __name__ == "__main__":
    while True:
        try:
            inp = input().rstrip("\n")
        except:
            exit(0)
        sret = nalifier.ShellInput(inp)
        if sret is None:
            if nalifier.Events:
                for event in nalifier.Events:
                    ProcessNARret(NAR.AddInput(event, Print=True))
            nalifier.Events = []
        else:
            #nalifier.ShellInput("*PROPERTY_OF_INTEREST=")
            ProcessNARret(NAR.AddInput(inp, Print=True))
