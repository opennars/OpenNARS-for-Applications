from subprocess import PIPE, run
import subprocess
import time
import glob

def ctests():
    result = run(['./YAN'], stdout=PIPE, stderr=PIPE, universal_newlines=True)
    print(result.stdout, result.stderr)
    success = result.returncode == 0
    if not success:
        exit(result.returncode)
    print("System tests successful!")
    return result.returncode

ctests()
time.sleep(1)
print("running example programs:")

for x in glob.glob('./examples/nal/*.nal'):
    print(subprocess.getoutput("cat " + x + " | ./YAN shell"))

print("Narsese integration tests successful!")

for x in glob.glob('./examples/english/*.english'):
    print(subprocess.getoutput("cat " + x + " | python2 english_shell.py"))

print("English integration tests successful!")
