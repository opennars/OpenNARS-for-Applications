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
mv src/main.c src/main_

echo "Compilation started:"
BaseFlags="-D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -mfpmath=sse -msse2 -pthread -lpthread -lm"
NoWarn="-Wno-unused-parameter"

Str=`ls src/*.c src/NetworkNAR/*.c | xargs`
gcc -c $NoWarn $BaseFlags $Str
ar rcs libONA.a *.o
gcc -c -fPIC $NoWarn $BaseFlags $Str
gcc -shared -o libONA.so *.o
rm -rf *.o

echo "Installing libONA in [ $install_dir ]."

sudo mkdir $install_dir/include/ona/
sudo cp src/*.h $install_dir/include/ona/
sudo mkdir $install_dir/include/ona/NetworkNAR/
sudo cp src/NetworkNAR/*.h $install_dir/include/ona/NetworkNAR/
sudo mv *.a $install_dir/lib/
sudo mv *.so $install_dir/lib/
mv src/main_ src/main.c

echo "Done."
