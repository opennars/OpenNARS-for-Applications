./build.sh
./NAR shell < ./examples/nal/activequestion.nal > output1
sort output1 > output1s
./build2.sh
./NAR shell < ./examples/nal/activequestion.nal > output2
sort output2 > output2s
diff output1s output2s > compilerdiff
