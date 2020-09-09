#!/bin/sh
rm NAR_first_stage *.o
rm src/RuleTable.c
mv src/main_ src/main.c
sudo rm -rf /usr/include/ona
sudo rm /usr/lib/libONA.a
sudo rm /usr/lib/libONA.so
Str=`ls src/*.c src/NetworkNAR/*.c | xargs`
echo $Str
echo "Compilation started:"
BaseFlags="-mfpmath=sse -msse2 -pthread -lpthread -lm -D_POSIX_C_SOURCE=199506L -pedantic -std=c99"
gcc -DSTAGE=1 -Wall -Wextra -Wformat-security $BaseFlags $Str -oNAR_first_stage
echo "First stage done, generating RuleTable.c now, and finishing compilation."
mv src/main.c src/main_
Str=`ls src/*.c src/NetworkNAR/*.c | xargs`
./NAR_first_stage NAL_GenerateRuleTable > ./src/RuleTable.c
gcc -c -DSTAGE=2 $BaseFlags $Str src/RuleTable.c
ar rcs libONA.a *.o
rm -rf *.o NAR_first_stage
gcc -c -fPIC -DSTAGE=2 $BaseFlags $Str src/RuleTable.c
gcc -shared -o libONA.so *.o
rm -rf *.o
sudo mkdir /usr/include/ona/
sudo cp src/*.h /usr/include/ona/
sudo mkdir /usr/include/ona/NetworkNAR/
sudo cp src/NetworkNAR/*.h /usr/include/ona/NetworkNAR/
sudo cp *.a /usr/lib/
sudo cp *.so /usr/lib/
mv src/main_ src/main.c
echo "Done."
