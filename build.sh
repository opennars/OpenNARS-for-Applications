Str=`ls src/*.c | xargs`
echo $Str 
gcc -DSDR_BLOCK_TYPE=__uint128_t -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 -Wall -Werror -Wno-format-security $Str -lm -oANSNA
