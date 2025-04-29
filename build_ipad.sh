#!/bin/sh
rm NAR
rm src/RuleTable.c
set -e
Str=`ls src/*.c | xargs`
echo $Str
echo "Compilation started:"
BaseFlags="-g -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 $Str -lm -oNAR"
NoWarn="-Wno-unknown-pragmas -Wno-tautological-compare -Wno-dollar-in-identifier-extension -Wno-unused-parameter -Wno-unused-variable -Wno-strict-prototypes"
gcc $@ -DSTAGE=1 -Wall -Wextra -Wformat-security $NoWarn $BaseFlags
echo "First stage done, generating RuleTable.c now, and finishing compilation."
./NAR NAL_GenerateRuleTable > ./src/RuleTable.c
export WASI_SDK_PATH=/home/tc/wasi-sdk
CC="${WASI_SDK_PATH}/bin/clang --sysroot=${WASI_SDK_PATH}/share/wasi-sysroot"
BaseFlags="-g -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 $Str -lm -oNAR.wasm"
/home/tc/wasi-sdk/build/install/opt/bin/clang $@ -DSTAGE=2 $NoWarn $BaseFlags src/RuleTable.c
echo "Done."
