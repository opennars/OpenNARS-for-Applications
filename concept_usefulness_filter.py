import sys
lines = []
for line in sys.stdin:
    lines.append(line)
lines = [line for line in lines if line.strip() != "" and ": { \"priority\":" in line]
lines.sort(key=lambda line: -float(line.split("\"usefulness\": ")[1].split(",")[0]))
displayN = len(lines) if len(sys.argv) <= 1 else int(sys.argv[1])
for (i,x) in enumerate(lines[0:displayN]):
    print("Comment: {i="+str(i)+"}", x, end='')
    
