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
import socket
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("-i", "--iterations", help="number of iterations to run", type=int)
args = parser.parse_args()
if args.verbose: debug = True
else: debug = False
if args.iterations: iterations = args.iterations
else: iterations = 1

ip = "127.0.0.1"
port = 50000

times = []
for it in range(iterations):
    # Set up socket for UDPNAR
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Read in toothbrush file and extract expected statements
    contents = ["*volume=0\0", "*motorbabbling=false\0"]
    expected = set()
    with open("./../../examples/nal/toothbrush.nal") as input_file:
        for line in input_file:
            if "//expected:" in line:
                expected.add(line.split("//expected: ")[1].rstrip())
            contents.append(line)

    # ./NAR UDPNAR IP PORT  timestep(ns per cycle) printDerivations
    process_cmd = ["./../../NAR", "UDPNAR", ip, str(port), "1000000", "true"]
    process = subprocess.Popen(process_cmd,
                               stdout=subprocess.PIPE,
                               universal_newlines=True)
    out_file = process.stdout
    # Check that the startup statements are present
    out_file.readline()
    out_file.readline()

    # Start timing when input begins to be provided
    start_time = time.time()

    # Iterate lines of input file, read from NARS when //expected is seen
    for line in contents:
        # If NARS expected to execute op, pause until seen
        if "//expected:" in line:
            target = line.split("//expected: ")[1].rstrip()
            if debug: print("EXPECTING: " + target)
            if debug: print("PAUSING INPUTS, READING NARS OUTPUT...")
            while target in expected:
                next_line = out_file.readline()
                # Uncomment below to capture all output from NARS, WILL BE A LOT OF TEXT
                #if debug: print(next_line)
                if target in next_line:
                    expected.remove(target)
                    if debug: print("TARGET FOUND IN: " + next_line.rstrip())
                    if debug: print("RESUMING INPUTS...")
        # Dont specify number of cycles, change volume/settings, or include comments
        if all(x.isdigit() for x in line[:-1]) or line[0] == "/":
            continue
        # If not waiting on expected, provide next input
        else:
            if debug: print("SENDING INPUTS: " + line.rstrip())
            sock.sendto((line.rstrip() + '\0').encode(), (ip, port))

    run_time = time.time() - start_time
    print("Iteration time: " + str(run_time))
    if debug: print("\n")
    times.append(run_time)
    process.terminate()

print("Average time: " + str(sum(times)/float(len(times))))
