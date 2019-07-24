Str=`ls src/*.c | xargs`
echo $Str
#To show unused code, add directly after gcc: 
gcc -ffunction-sections -fdata-sections -Wl,--gc-sections -Wl,--print-gc-sections -DSDR_BLOCK_TYPE=__uint128_t -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 -Wall -Werror -Wextra -Wformat-security $Str -lm -oANSNA
