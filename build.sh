Str=""
for i in ./src/*.c
do
    Str=$Str" ${i%.c}.c"
done
echo $Str
gcc -pedantic -std=c11 -g3 -o3 -Wall $Str -lm -oANSNA
