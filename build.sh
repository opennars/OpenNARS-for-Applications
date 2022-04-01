#!/bin/sh
rm NAR
set -e
Str=`ls src/*.c src/NetworkNAR/*.c | xargs`
echo $Str
echo "Compilation started:"
BaseFlags="-pthread -lpthread -D_POSIX_C_SOURCE=199506L -Wno-unused-parameter -pedantic -std=c99 -g3 -O3 $Str -lm -oNAR"
gcc $1 -Wall -Wextra -Wformat-security $BaseFlags
echo "Done."
