Str=`ls src/*.c | xargs`
echo $Str
echo "Compilation started: Unused code will be printed and removed from the binary:"
gcc -ffunction-sections -fdata-sections -Wl,--gc-sections -Wl,--print-gc-sections -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 -Wall -Wextra -Wformat-security $Str -lm -oYAN
./YAN NAL_GenerateRuleTable > ./src/RuleTable.c
echo "Done."
echo "Compile again on first compilation, or if changes to rule files have been made as well!"
