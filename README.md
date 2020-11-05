*Minimal Sensorimotor Component 2*

<div style="text-align:center"><img src="https://user-images.githubusercontent.com/8284677/98208148-be83c800-1f34-11eb-8ab4-d6a0eb0eef77.png" height="350"></div>

This is a NARS implementation which supports only NAL6-8 inference, it's a stripped down version of ONA v0.8.5

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

