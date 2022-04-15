#!/bin/sh
set -e

if [ -z "$CC" ]; then
    CC=gcc
fi

rm NAR
rm src/RuleTable.c
sources=`ls src/*.c src/NetworkNAR/*.c | xargs`
echo $sources
echo "Compilation started:"
BaseFlags="-g -pthread -lpthread -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 $sources -lm -oNAR"
NoWarn="-Wno-tautological-compare -Wno-dollar-in-identifier-extension -Wno-unused-parameter -Wno-unused-variable"

# check if we are on Windows, use SSE2 then
if [ "$OSTYPE" == "cygwin" ]; then
    echo "Compiling for Windows (x86/x64)."
    # CC=i686-w64-mingw32-gcc
    BaseFlags="-mfpmath=sse -msse2 $BaseFlags"
fi

# if CPU architecture is x86/x64, then use SSE2
if [[ "$OSTYPE" != "cygwin" && "$(uname -m)" == "x86"* ]]; then
    echo "Compiling for x86/x64."
    BaseFlags="-mfpmath=sse -msse2 $BaseFlags"
fi

$CC -DSTAGE=1 -Wall -Wextra -Wformat-security $NoWarn $BaseFlags
echo "First stage done, generating RuleTable.c now, and finishing compilation."
./NAR NAL_GenerateRuleTable > ./src/RuleTable.c

if [[ "$*" == *"-fopenmp"* ]]; then
    echo "Building with OpenMP support."
fi

$CC -DSTAGE=2 $NoWarn $BaseFlags $@ src/RuleTable.c
echo "Done."
