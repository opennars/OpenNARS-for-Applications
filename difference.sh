./build.sh
./NAR shell < ./examples/nal/example1.nal > output1
sort output1 > output1s
./build2.sh
./NAR shell < ./examples/nal/example1.nal > output2
sort output2 > output2s
diff output1s output2s > sortedDiff
diff output1 output2 > Diff
