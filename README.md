<div style="text-align:center"><img src="https://user-images.githubusercontent.com/8284677/74610111-0c774800-50e8-11ea-8592-102ce692d5e8.png" height="250"></div>

This is a completely new platform and not branched from the existing OpenNARS codebase. The ONA (OpenNARS for Applications) system takes the logic and conceptual ideas of OpenNARS, the event handling and procedure learning capabilities of ANSNA, and the control model from ALANN. The system is written in C and is extremely capable. 

The ONA implementation has been developed with a pragmatic mindset. The focus on the design has been to implement the 'existing' theory as effectively as possible and make firm decisions rather than keep as many options open as possible. This has led to some small conceptual differences to OpenNARS for Research (ONR). 


***How to clone and compile (tested with GCC and Clang for x64, x86 and ARM):***

```
git clone https://github.com/opennars/OpenNARS-for-Applications
cd OpenNARS-for-Applications
./build.sh
```

***How to set the amount of threads the system should run with: (to be tested more, compile with ./build.sh -fopenmp)***
```
export OMP_NUM_THREADS=4  // 4 threads seems to be the sweet spot. More threads leads to more contention and less speed
```

***How to run the interactive Narsese shell:***

```
./NAR shell
```

***How to run the C tests and then receive instructions how to run the current example programs:***

```
./NAR
```

***How to run all C tests, and all Narsese and English examples as integration tests, and collect metrics across all examples:***

```
python3 evaluation.py
```

For the current output, see [Evaluation results](https://github.com/opennars/OpenNARS-for-Applications/wiki/Evaluation-Results-(Tests,-metrics))

**How to run an example file:**

Narsese:

```
./NAR shell < ./examples/nal/example1.nal
```

English: (needs NLTK v3.4.5, will be updated to Python3 at some point as well...)

```
python2 english_shell.py < ./examples/english/story1.english
```

**How to run an UDPNAR:**

```
./NAR UDPNAR IP PORT timestep(ns per cycle) printDerivations
./NAR UDPNAR 127.0.0.1 50000 10000000 true
```

where the output can be logged simply by appending

```
> output.log
```

**How to reach us:**

Real-time team chat: #nars IRC channel @ freenode.net, #nars:matrix.org (accessible via Riot.im)

Google discussion group: https://groups.google.com/forum/#!forum/open-nars
