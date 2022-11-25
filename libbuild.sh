#!/bin/sh
lib_dir="/usr/local/lib"
include_dir="/usr/local/include"

if [ $# -eq 2 ]; then
    echo "[ALERT] The defauld base_dir [$lib_dir $include_dir] is changed in $1."
    lib_dir=$1
    include_dir=$2
fi

rm NAR_first_stage *.o
rm src/RuleTable.c
mv src/main_ src/main.c
sudo rm -rf $include_dir/ona
sudo rm $lib_dir/libONA.*
Str=`ls src/*.c src/NetworkNAR/*.c | xargs`

echo "Compilation started:"
BaseFlags="-D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -pthread -lpthread -lm"
NoWarn="-Wno-unknown-pragmas -Wno-tautological-compare -Wno-dollar-in-identifier-extension -Wno-unused-parameter -Wno-unused-variable"
gcc -mfpmath=sse -msse2 -DSTAGE=1 -Wall -Wextra -Wformat-security $Str $NoWarn $BaseFlags -oNAR_first_stage || gcc -DSTAGE=1 -Wall -Wextra -Wformat-security $Str $NoWarn $BaseFlags -oNAR_first_stage
echo "First stage done, generating RuleTable.c now, and finishing compilation."

mv src/main.c src/main_
Str=`ls src/*.c src/NetworkNAR/*.c | xargs`
./NAR_first_stage NAL_GenerateRuleTable > ./src/RuleTable.c
gcc -mfpmath=sse -msse2 -c -DSTAGE=2 $NoWarn $BaseFlags $Str src/RuleTable.c || gcc -c -DSTAGE=2 $NoWarn $BaseFlags $Str src/RuleTable.c
ar rcs libONA.a *.o
rm -rf *.o NAR_first_stage
gcc -mfpmath=sse -msse2 -c -fPIC -DSTAGE=2 $NoWarn $BaseFlags $Str src/RuleTable.c || gcc -c -fPIC -DSTAGE=2 $NoWarn $BaseFlags $Str src/RuleTable.c
gcc -shared -o libONA.so *.o
rm -rf *.o

echo "Installing libONA in [ $lib_dir $include_dir ]."

sudo mkdir $include_dir/ona/
sudo cp src/*.h $include_dir/ona/
sudo mkdir $include_dir/ona/NetworkNAR/
sudo cp src/NetworkNAR/*.h $include_dir/ona/NetworkNAR/
sudo mv *.a $lib_dir/
sudo mv *.so $lib_dir/
mv src/main_ src/main.c

echo "Done."
