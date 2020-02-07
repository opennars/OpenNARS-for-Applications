**Yet Another NARS**

![YAN](https://user-images.githubusercontent.com/8284677/71787964-c96c8400-3015-11ea-91ac-2b98d621be33.png)

A complete and mature next generation NARS implementation in C, built on top of Minimal Sensorimotor Component (a stripped down ANSNA with Compound Terms instead of SDR's, see https://github.com/patham9/MSC)
Differently than OpenNARS, YAN focuses on inference control and overall reliability. It is an attempt to close the loop from sensor input to goal-driven action in such a way that advanced applications become possible. The most crucial NAL inference rules are implemented in the system, while experimental NARS features (such as NAL-9 and self-regulation) are not part of YAN, and neither are inference rules which are hard to control, such as multi-conditional syllogisms. YAN sacrifices some flexibility for extreme reliability, its strengths lie in sensorimotor (control tasks, robotics), story understanding, real-time applications and its ability to run uninterrupted.
Additionally, the system's memory footprint can be strictly limited, allowing it to be used in embedded systems. It has already been tested on various ARM processors.

***How to clone and compile (tested with GCC and Clang for x64, x86 and ARM):***

```
git clone https://github.com/patham9/YAN
cd YAN
./build.sh
```

***How to set the amount of threads the system should run with:***
```
export OMP_NUM_THREADS=8
```

***How to run the interactive Narsese shell:***

```
./YAN shell
```

***How to run the C tests and then receive instructions how to run the current example programs:***

```
./YAN
```

***How to run all C tests, and all Narsese and English examples as integration tests, and collect metrics across all examples:***

```
python3 evaluation.py
```

For the current output, see [Evaluation results](https://github.com/patham9/YAN/wiki/Evaluation-Results-(Tests,-metrics))

**How to run an example file:**

Narsese:

```
./YAN shell < ./examples/nal/example1.nal
```

English: (will be updated to Python3 at some point as well...)

```
python2 english_shell.py < ./examples/english/story1.english
```

**Other**

Usage from IRC is also possible, see https://gist.github.com/patham9/75b53ea120f140fce6538271e217dfac
