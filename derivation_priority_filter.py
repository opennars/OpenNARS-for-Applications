import sys
if len(sys.argv) <= 1:
    print("DerivationPriorityFilter.py, error: has to be called with priority threshold: python3 PriorityFilter.py threshold")
    exit(1)
threshold = float(sys.argv[1])
for line in sys.stdin:
    if "Priority=" in line:
        prio = float(line.split("Priority=")[1].split(" ")[0])
        if prio < threshold:
            continue
    print(line, end='')
    sys.stdout.flush()
