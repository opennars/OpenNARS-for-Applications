Str=""
for i in *.c
do
    Str=$Str" ${i%.c}.c"
done
gcc -g3 -o3 $Str
