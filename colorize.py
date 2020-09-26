import sys
import re

GREEN = "\x1B[32m"
YELLOW = "\x1B[33m"
CYAN = "\x1B[36m"
MAGENTA = "\x1B[35m"
RED = "\x1B[31m"
RESET = "\x1B[0m"
BOLD = "\x1B[1m"

for line in sys.stdin:
    line = line.rstrip()
    COLOR = "\x1B[0m"
    if line.startswith("performing ") or line.startswith("done with"):
        COLOR = CYAN
    if line.startswith("Comment:"):
        COLOR = MAGENTA
    if line.startswith("Input:"):
        COLOR = GREEN
    if line.startswith("Derived:"):
        COLOR = YELLOW
    if line.startswith("Answer:") or line.startswith("^") or "decision expectation" in line:
        COLOR = BOLD + RED
    #Ext and Int set
    l = re.sub(r"{([^><:]*)}", MAGENTA+r"{" + GREEN + r"\1" + MAGENTA + "}" + COLOR, line)
    l = re.sub(r"\[([^><:]*)\]", MAGENTA+r"[" + GREEN + r"\1" + MAGENTA + "]" + COLOR, l)
    #Image
    l = re.sub(r"\(([^><:]*)\s(/1|\\1|/2|\\1|\\2)\s([^><:]*)\)", YELLOW+r"(" + GREEN + r"\1" + YELLOW + r" \2 " + GREEN + r"\3" + YELLOW + ")" + COLOR, l)
    #Implication
    l = re.sub(r"<([^:]*)\s=/>\s([^:]*)>", CYAN+r"<" + GREEN + r"\1" + CYAN + r" =/> " + GREEN + r"\2" + CYAN + ">" + COLOR, l)
    #Sequence
    l = re.sub(r"\(([^:]*)\s&/\s([^:]*)\)", MAGENTA+r"(" + GREEN + r"\1" + MAGENTA + r" &/ " + GREEN + r"\2" + MAGENTA + ")" + COLOR, l)
    #Similarity and inheritance
    l = re.sub(r"<([^><:]*)\s((-->)|(<->))\s([^><:]*)>", RED+r"<" + GREEN + r"\1" + RED + r" \2 " + GREEN + r"\5" + RED + ">" + COLOR, l)
    #Other compound term copulas (not higher order)
    l = re.sub(r"\(([^><:]*)\s(\*|&|~|-|\|)\s([^><:]*)\)", YELLOW+r"(" + GREEN + r"\1" + YELLOW + r" \2 " + GREEN + r"\3" + YELLOW + ")" + COLOR, l)
    print(COLOR + l + RESET)
