Str=""
for i in ./src/*.c
do
    Str=$Str" ${i%.c}.c"
done
echo $Str
gcc -g3 -o3 -Wall -Wmissing-prototypes -Wmissing-declarations  -Werror=missing-prototypes $Str -oANSNA
