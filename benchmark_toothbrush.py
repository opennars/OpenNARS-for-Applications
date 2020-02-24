import subprocess
import socket
import time
import os

contents = []
expected = set()
with open("examples/nal/toothbrush.nal") as input_file:
    for line in input_file:
        if "//expected:" in line:
            expected.add(line.split("//expected: ")[1])
        contents.append(line)

ip = "127.0.0.1"
port = 50000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ./NAR UDPNAR IP PORT  timestep(ns per cycle) printDerivations
process_cmd = ["./NAR", "UDPNAR", ip, str(port), "1000000", "true"]
process = subprocess.Popen(process_cmd,
                           stdout=subprocess.PIPE,
                           stdin=None,
                           shell=True,
                           universal_newlines=True)
out_file = process.stdout
s = os.fstat(out_file.fileno()).st_size
print(s)
print(out_file.readline(), flush=True)
time.sleep(2)
poll = process.poll()
if poll == None: print("still alive")
else: print("ded")
exit()

for line in contents:
    # Doing live run, don't specify cycles
    if line[:-1].isnumeric():
        print("skipping " + line)
        continue
    # If NARS expected to provide action, don't proceed until seen
    if "//expected:" in line:
        target = line.split("//expected: ")[1]
        while target in expected:
            next_line = out_file.readline()
            to_remove = None
            # Check against all expected statements in case it derives one of the future requirements early
            for expected_elem in expected:
                if expected_elem in next_line:
                    to_remove = expected_elem
                    print(next_line)
            if to_remove:
                expected.remove(to_remove)
    # If no statement expected, provide next input
    else:
        print(line)
        sock.sendto(line.rstrip().encode(), (ip, port))

# Flush out last statement
out_file.readline()
in_file.close()

# Get finishing statements
while True:
    next_line = out_file.readline()
    print(next_line)
    if next_line == "": break
    
