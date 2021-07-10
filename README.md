<div style="text-align:center"><img src="https://user-images.githubusercontent.com/8284677/74609985-02087e80-50e7-11ea-9562-218dec34714d.png" height="250"></div>

Implementation of a Non-Axiomatic Reasoning System [6], a general-purpose reasoner that adapts under the Assumption of Insufficient Knowledge and Resources [7].

This is a completely new platform and not branched from the existing OpenNARS codebase. The ONA (OpenNARS for Applications) system [1] takes the logic and conceptual ideas of OpenNARS, the event handling and procedure learning capabilities of ANSNA [2, 3], and the control model from ALANN [4]. The system is written in C, is more capable than our previous implementations, and has also been experimentally compared with Reinforcement Learning [5]. 

The ONA implementation has been developed with a pragmatic mindset. The focus on the design has been to implement the 'existing' theory [6, 7] as effectively as possible and make firm decisions rather than keep as many options open as possible. This has led to some small conceptual differences to OpenNARS [8] which was developed for research purposes. 

Video tutorials and demo videos can be found here: [Video tutorials](https://github.com/opennars/OpenNARS-for-Applications/wiki/Video-tutorials)

Or click the picture to watch the newest summary video:

[![OpenNARS for Applications v0.8.8: Demos](https://img.youtube.com/vi/oyQ250H5owE/0.jpg)](https://www.youtube.com/watch?v=oyQ250H5owE "OpenNARS for Applications v0.8.8: Demos")

***How to clone and compile (tested with GCC and Clang for x64, x86 and ARM):***

```
git clone https://github.com/opennars/OpenNARS-for-Applications
cd OpenNARS-for-Applications
./build.sh
```

***How to set the amount of threads the system should run with: (to be tested more, compile with ./build.sh -fopenmp)***
```
export OMP_NUM_THREADS=4  // 4 threads seems to be the sweet spot. More threads leads to more contention and less speed currently
```

***How to run the interactive Narsese shell:***

```
./NAR shell
```

***with syntax highlighting:***

```
./NAR shell | python3 colorize.py
```

***with English NLP shell and syntax highlighting:***

```
python3 english_to_narsese.py | ./NAR shell | python3 colorize.py
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

English: (tested with NLTK v3.4.5, v3.5)

```
python3 english_to_narsese.py < ./examples/english/story1.english | ./NAR shell
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

**References**

[1] Hammer, P., & Lofthouse, T. (2020, September). ‘OpenNARS for Applications’: Architecture and Control. In International Conference on Artificial General Intelligence (pp. 193-204). Springer, Cham.

[2] Hammer, P. (2019, August). Adaptive Neuro-Symbolic Network Agent. In International Conference on Artificial General Intelligence (pp. 80-90). Springer, Cham.

[3] Hammer, P., & Lofthouse, T. (2018, August). Goal-directed procedure learning. In International Conference on Artificial General Intelligence (pp. 77-86). Springer, Cham.

[4] Lofthouse, T. (2019). ALANN: An event driven control mechanism for a non-axiomatic reasoning system (NARS).

[5] Eberding, L. M., Thórisson, K. R., Sheikhlar, A., & Andrason, S. P. (2020). SAGE: Task-Environment Platform for Evaluating a Broad Range of AI Learners. In Artificial General Intelligence: 13th International Conference, AGI 2020, St. Petersburg, Russia, September 16–19, 2020, Proceedings (Vol. 12177, p. 72). Springer Nature.

[6] Wang, P. (2013). Non-axiomatic logic: A model of intelligent reasoning. World Scientific.

[7] Wang, P. (2009, October). Insufficient Knowledge and Resources-A Biological Constraint and Its Functional Implications. In AAAI Fall Symposium: Biologically Inspired Cognitive Architectures.

[8] Hammer, P., Lofthouse, T., & Wang, P. (2016, July). The OpenNARS implementation of the non-axiomatic reasoning system. In International conference on artificial general intelligence (pp. 160-170). Springer, Cham.

