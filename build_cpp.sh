#!/bin/sh
rm NAR
rm src/RuleTable.c
set -e
Str=`ls src/*.c | xargs`
echo $Str
echo "Compilation started:"
BaseFlags="-g -D_POSIX_C_SOURCE=199506L -std=c++20 -g3 -O3 $Str -lm -oNAR"
NoWarn="-Wno-unknown-pragmas -Wno-tautological-compare -Wno-dollar-in-identifier-extension -Wno-unused-parameter -Wno-unused-variable -Wno-write-strings -Wno-missing-field-initializers -Wno-narrowing"
g++ $@ -DSTAGE=1 -Wall -Wextra -Wformat-security $NoWarn $BaseFlags
echo "First stage done, generating RuleTable.c now, and finishing compilation."
./NAR NAL_GenerateRuleTable > ./src/RuleTable.c
g++ $@ -mfpmath=sse -msse2 -DSTAGE=2 $NoWarn $BaseFlags src/RuleTable.c || (echo "Error with SSE, hence compiling without SSE:" && g++ $@ -DSTAGE=2 $NoWarn $BaseFlags src/RuleTable.c)
echo "Done."
