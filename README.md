**OpenNARS for Applications**

<img src="https://user-images.githubusercontent.com/8284677/74609985-02087e80-50e7-11ea-9562-218dec34714d.png" width="500" height="450">

This is a completely new platform and not branched from the existing OpenNARS codebase. The ONA (OpenNARS for Applications) system takes the logic and conceptual ideas of OpenNARS, the event handling and procedure learning capabilities of ANSNA, and the control model from ALANN. The system is written in C and is extremely capable. 

The ONA implementation has been developed with a pragmatic mindset. The focus on the design has been to implement the 'existing' theory as effectively as possible and make firm decisions rather than keep as many options open as possible. This has led to some small conceptual differences to OpenNARS for Research (ONR). 


***How to clone and compile (tested with GCC and Clang for x64, x86 and ARM):***

```
git clone https://github.com/opennars/OpenNARS-for-Applications
cd OpenNARS-for-Applications
./build.sh
```

***How to set the amount of threads the system should run with: (experimental, compile with ./build.sh -fopenmp)***
```
export OMP_NUM_THREADS=8
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

English: (will be updated to Python3 at some point as well...)

```
python2 english_shell.py < ./examples/english/story1.english
```
