Str=""
for i in ./src/*.c
do
    Str=$Str" ${i%.c}.c"
done
echo $Str
gcc -DSDR_BLOCK_TYPE=__uint128_t -pedantic -std=c99 -g3  -O3  -Wall $Str -lm -oANSNA
