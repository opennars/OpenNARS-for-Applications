import sys
import re

GREEN = "\x1B[32m"
YELLOW = "\x1B[33m"
CYAN = "\x1B[36m"
MAGENTA = "\x1B[35m"
RED = "\x1B[31m"
RESET = "\x1B[0m"
BOLD = "\x1B[1m"
STATEMENT_OPENER = r"<" if len(sys.argv) <= 1 or sys.argv[1] == "old" else r"("
STATEMENT_CLOSER = r">" if len(sys.argv) <= 1 or sys.argv[1] == "old" else r")"

for line in sys.stdin:
    line = line.rstrip()
    COLOR = GREEN
    if line.startswith("performing ") or line.startswith("done with"):
        COLOR = CYAN
    if line.startswith("Comment:") or line.startswith("//"):
        COLOR = MAGENTA
    if line.startswith("Input:"):
        COLOR = GREEN
    if line.startswith("Derived:"):
        COLOR = YELLOW
    if line.startswith("Answer:") or line.startswith("^") or "decision expectation" in line:
        COLOR = BOLD + RED
    #Ext and Int set
    l = re.sub(r"{([^><:\(\)\*]*)}", MAGENTA+r"{" + GREEN + r"\1" + MAGENTA + "}" + COLOR, line)
    l = re.sub(r"\[([^><:\(\)\*]*)\]", MAGENTA+r"[" + GREEN + r"\1" + MAGENTA + "]" + COLOR, l)
    #Image
    l = re.sub(r"\(([^><:]*)\s(/1|\\1|/2|\\1|\\2)\s([^><:]*)\)", YELLOW+r"(" + GREEN + r"\1" + YELLOW + r" \2 " + GREEN + r"\3" + YELLOW + ")" + COLOR, l)
    #Implication
    l = re.sub(r"<([^:]*)\s=(/|=|\|)>\s([^:]*)>", CYAN+ STATEMENT_OPENER + GREEN + r"\1" + CYAN + r" =\2> " + GREEN + r"\3" + CYAN + STATEMENT_CLOSER + COLOR, l)
    #Conjunction
    l = re.sub(r"\(([^:]*)\s&(/|&|\|)\s([^:]*)\)", MAGENTA+r"(" + GREEN + r"\1" + MAGENTA + r" &\2 " + GREEN + r"\3" + MAGENTA + ")" + COLOR, l)
    #Similarity and inheritance
    l = re.sub(r"<([^><:]*)\s((-->)|(<->))\s([^><:]*)>", RED+ STATEMENT_OPENER + GREEN + r"\1" + RED + r" \2 " + GREEN + r"\5" + RED + STATEMENT_CLOSER + COLOR, l)
    #Other compound term copulas (not higher order)
    l = re.sub(r"\(([^><:]*)\s(\*|&|~|-|\|)\s([^><:]*)\)", YELLOW+r"(" + GREEN + r"\1" + YELLOW + r" \2 " + GREEN + r"\3" + YELLOW + ")" + COLOR, l)
    print(COLOR + l + RESET)
