import subprocess
import socket
import time
import os

contents = []
expected = set()
with open("examples/nal/toothbrush.nal") as input_file:
    for line in input_file:
        if "//expected:" in line:
            expected.add(line.split("//expected: ")[1].rstrip())
        contents.append(line)

ip = "127.0.0.1"
port = 50000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ./NAR UDPNAR IP PORT  timestep(ns per cycle) printDerivations
process_cmd = ["./NAR", "UDPNAR", ip, str(port), "10000000", "true"]
process = subprocess.Popen(process_cmd,
                           stdout=subprocess.PIPE,
                           universal_newlines=True)
out_file = process.stdout

for line in contents:
    # If NARS expected to provide action, don't proceed until seen
    if "//expected:" in line:
        target = line.split("//expected: ")[1].rstrip()
        print("LOOKING FOR STRING: " + target)
        print("READING NARS OUTPUT\n===================")
        while target in expected:
            next_line = out_file.readline()
            print(next_line)
            to_remove = None
            # Check against all expected statements in case it derives one of the future requirements early
            for expected_elem in expected:
                if expected_elem in next_line:
                    to_remove = expected_elem
                    #print(next_line)
            if to_remove:
                expected.remove(to_remove)
    # Doing live run, don't specify cycles, currently does not handle comments or settings
    if line[:-1].isnumeric() or line[0] == "*" or line[0] == "/":
        continue
    # If no statement expected, provide next input
    else:
        print("SENDING INPUTS: " + line)
        sock.sendto(line.rstrip().encode(), (ip, port)) 
