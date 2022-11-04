set Str=src/Cycle.c src/Decision.c src/Event.c src/Globals.c src/HashTable.c src/Inference.c src/InvertedAtomIndex.c src/main.c src/Memory.c src/NAL.c src/NAR.c src/Narsese.c src/PriorityQueue.c src/Shell.c src/Stack.c src/Stamp.c src/Stats.c src/Table.c src/Term.c src/Truth.c src/Usage.c src/Variable.c
echo %Str%
echo "Compilation started:"
set NoWarn=-Wno-unknown-pragmas -Wno-tautological-compare -Wno-dollar-in-identifier-extension -Wno-unused-parameter -Wno-unused-variable -Wno-write-strings -Wno-missing-field-initializers -Wno-narrowing
C:/tcc/tcc -DSTAGE=1 -Wall -Wextra -Wformat-security %NoWarn% %Str% -oNAR.exe
echo "First stage done, generating RuleTable.c now, and finishing compilation."
NAR NAL_GenerateRuleTable > src/RuleTable.c
C:/tcc/tcc -DSTAGE=2 %Str% src/RuleTable.c -oNAR.exe
echo "Done."
