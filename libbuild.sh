#!/bin/sh
install_dir="/usr/local"

if [ $# -eq 1 ]; then
    echo "[ALERT] The defauld base_dir [$install_dir] is changed in $1."
    install_dir=$1
fi

rm NAR_first_stage *.o
rm src/RuleTable.c
mv src/main_ src/main.c
sudo rm -rf $install_dir/include/ona
sudo rm $install_dir/lib/libONA.*
Str=`ls src/*.c src/NetworkNAR/*.c | xargs`

echo "Compilation started:"
BaseFlags="-D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -pthread -lpthread -lm"
NoWarn="-Wno-tautological-compare -Wno-dollar-in-identifier-extension -Wno-unused-parameter -Wno-unused-variable"
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

echo "Installing libONA in [ $install_dir ]."

sudo mkdir $install_dir/include/ona/
sudo cp src/*.h $install_dir/include/ona/
sudo mkdir $install_dir/include/ona/NetworkNAR/
sudo cp src/NetworkNAR/*.h $install_dir/include/ona/NetworkNAR/
sudo mv *.a $install_dir/lib/
sudo mv *.so $install_dir/lib/
mv src/main_ src/main.c

echo "Done."
