Str=`ls src/*.c | xargs`
echo $Str
echo "Compilation started: Unused code will be printed and removed from the binary:"
compileparams="-ffunction-sections -fdata-sections -Wl,--gc-sections -Wl,--print-gc-sections -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 -Wall -Wextra -Wformat-security $Str -lm -oYAN"
gcc $compileparams
./YAN NAL_GenerateRuleTable > ./src/RuleTable.c
echo "Done."
