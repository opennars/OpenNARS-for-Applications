import subprocess
proc = subprocess.Popen(['./NAR', 'shell'],
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)

# To avoid deadlocks: careful to: add \n to output, flush output, use
# readline() rather than read()
proc.stdin.write(b'a-->b\n')
proc.stdin.flush()
print(proc.stdout.readline())


proc.stdin.close()
proc.terminate()
proc.wait(timeout=0.2)
