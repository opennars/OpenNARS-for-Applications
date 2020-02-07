rm YAN
rm src/RuleTable.c
set -e
Str=`ls src/*.c | xargs`
echo $Str
echo "Compilation started: Unused code will be printed and removed from the binary:"
BaseFlags="-D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 $Str -lm -oYAN"
gcc -DSTAGE=1 -Wall -Wextra -Wformat-security $BaseFlags
echo "First stage done, generating RuleTable.c now, and finishing compilation."
./YAN NAL_GenerateRuleTable > ./src/RuleTable.c
gcc $1 -ffunction-sections -fdata-sections -Wl,--gc-sections -Wl,--print-gc-sections -DSTAGE=2 $BaseFlags src/RuleTable.c
echo "Done."
