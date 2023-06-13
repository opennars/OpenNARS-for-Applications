<div style="text-align:center"><img src="https://user-images.githubusercontent.com/8284677/74609985-02087e80-50e7-11ea-9562-218dec34714d.png" height="250"></div>

Implementation of a Non-Axiomatic Reasoning System [6], a general-purpose reasoner that adapts under the Assumption of Insufficient Knowledge and Resources [7].

This is a completely new platform and not branched from the existing OpenNARS codebase. The ONA (OpenNARS for Applications) system [1] takes the logic and conceptual ideas of OpenNARS, the event handling and procedure learning capabilities of ANSNA [2, 3] and 20NAR1 [11], and the control model from ALANN [4]. The system is written in C, is more capable than our previous implementations in terms of reasoning performance, and has also been experimentally compared with Reinforcement Learning [5, 6] and means-end reasoning approaches such as BDI models [6]. Additionally, it has become the core reasoning component of a system assisting first responders (Trusted and explainable Artificial Intelligence for Saving Lives, [6]) while driving and completing their mission. This was done in cooperation with NASA Jet Propulsion Laboratory. Also it has been tried for real-time traffic surveillance in cooperation with Cisco Systems [7]. Last, initial experiments for using the system for autonomous robots have been carried out [6], and more is yet to come.

The ONA implementation has been developed with a pragmatic mindset. The focus on the design has been to implement the 'existing' theory [8, 9] as effectively as possible and make firm decisions rather than keep as many options open as possible. This has led to some small conceptual differences to OpenNARS [10] which was developed for research purposes. 

Video tutorials and demo videos can be found here: [Video tutorials](https://github.com/opennars/OpenNARS-for-Applications/wiki/Video-tutorials)
Or click on the picture to watch the newest summary videos (summary and demo):

[![Reasoning-learning systems based on Non Axiomatic Reasoning Theory](https://img.youtube.com/vi/pEiJ8V17RGk/0.jpg)](https://www.youtube.com/watch?v=pEiJ8V17RGk "Reasoning-learning systems based on Non Axiomatic Reasoning Theory")

[![Autonomy through real-time learning and OpenNARS for Applications](https://img.youtube.com/vi/B9SKu7u6G-I/0.jpg)](https://www.youtube.com/watch?v=B9SKu7u6G-I "Autonomy through real-time learning and OpenNARS for Applications")

[![OpenNARS for Applications v0.9.0: Transbot](https://img.youtube.com/vi/lp6rNO-nIms/0.jpg)](https://www.youtube.com/watch?v=lp6rNO-nIms "ONA v0.9.0: Playing Fetch with Henry the robot")

Procedure learning demos (variants of Pong and Space Invaders, Test Chamber, Cartpole, food collecting agent, ...): https://www.youtube.com/watch?v=oyQ250H5owE

***How to clone and compile (tested with GCC and Clang for x64, x86 and ARM):***

```
git clone https://github.com/opennars/OpenNARS-for-Applications
cd OpenNARS-for-Applications
./build.sh
```

Additionally the parameter -DHARDENED can be passed to build.sh to end up with a slimmer system without language learning abilities.

***How to set the amount of threads the system should run with: (to be tested more, compile with ./build.sh -fopenmp)***
```
export OMP_NUM_THREADS=4  // 4 threads seems to be the sweet spot. More threads leads to more contention and less speed currently
```

If you have trouble building with OpenMP, then you probably need to specify library (and / or sources) directory alongside the `-fopenmp` option, like `-L<path to your openmp>` or `-I<path to your openmp>`.

***How to run the interactive Narsese shell:***

```
./NAR shell
```

***with syntax highlighting:***

```
./NAR shell | python3 colorize.py
```

***For a proper reliable GPT-based English language channel***

Check out [NARS-GPT](https://github.com/opennars/NARS-GPT) <img src="https://user-images.githubusercontent.com/8284677/234757994-5e8ad001-c5b1-4aa1-abe7-c56a4f7012dd.png" width="50px">!

***with legacy English NLP shell and syntax highlighting:***

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

Real-time team chat: #nars IRC channel @ libera.chat, #nars:matrix.org (accessible via Riot.im)

Google discussion group: https://groups.google.com/forum/#!forum/open-nars

**References**

[1] Hammer, P., & Lofthouse, T. (2020, September). [‘OpenNARS for Applications’: Architecture and Control](https://www.researchgate.net/publication/342713626_%27OpenNARS_for_Applications%27_Architecture_and_Control). In International Conference on Artificial General Intelligence (pp. 193-204). Springer, Cham.

[2] Hammer, P. (2019, August). [Adaptive Neuro-Symbolic Network Agent](http://agi-conf.org/2019/wp-content/uploads/2019/07/paper_15.pdf). In International Conference on Artificial General Intelligence (pp. 80-90). Springer, Cham.

[3] Hammer, P., & Lofthouse, T. (2018, August). [Goal-directed procedure learning](https://www.researchgate.net/publication/326525686_Goal-Directed_Procedure_Learning_11th_International_Conference_AGI_2018_Prague_Czech_Republic_August_22-25_2018_Proceedings). In International Conference on Artificial General Intelligence (pp. 77-86). Springer, Cham.

[4] Lofthouse, T. (2019). [ALANN: An event driven control mechanism for a non-axiomatic reasoning system (NARS)](https://cis.temple.edu/tagit/events/papers/Lofthouse.pdf). NARS2019 workshop at AGI 2019.

[5] Eberding, L. M., Thórisson, K. R., Sheikhlar, A., & Andrason, S. P. (2020). [SAGE: Task-Environment Platform for Evaluating a Broad Range of AI Learners](http://alumni.media.mit.edu/~kris/ftp/SAGE__Task_Environment_Platform_for_Evaluating_a_Broad_Range_of_AI_Learners.pdf). In Artificial General Intelligence: 13th International Conference, AGI 2020, St. Petersburg, Russia, September 16–19, 2020, Proceedings (Vol. 12177, p. 72). Springer Nature.

[6] Hammer, P. (2021, July). [Autonomy through real-time learning and OpenNARS for Applications](https://github.com/opennars/OpenNARS-for-Applications/files/6832325/Dissertation_PH_Submitted.pdf). PhD thesis at Department of Computer and Information Sciences, Temple Universitiy

[7] Hammer, P., Lofthouse, T., Fenoglio, E., Latapie, H., & Wang, P. (2020, September). [A reasoning based model for anomaly detection in the Smart City domain](https://www.researchgate.net/publication/335444390_A_reasoning_based_model_for_anomaly_detection_in_the_Smart_City_domain). In Proceedings of SAI Intelligent Systems Conference (pp. 144-159). Springer, Cham.

[8] Wang, P. (2013). [Non-axiomatic logic: A model of intelligent reasoning](https://www.worldscientific.com/worldscibooks/10.1142/8665). World Scientific.

[9] Wang, P. (2009, October). [Insufficient Knowledge and Resources-A Biological Constraint and Its Functional Implications](https://cis.temple.edu/~pwang/Publication/AIKR.pdf). In AAAI Fall Symposium: Biologically Inspired Cognitive Architectures.

[10] Hammer, P., Lofthouse, T., & Wang, P. (2016, July). [The OpenNARS implementation of the non-axiomatic reasoning system](https://cis.temple.edu/~pwang/Publication/OpenNARS.pdf). In International conference on artificial general intelligence (pp. 160-170). Springer, Cham.

[11] Wünsche, R. (2021, October). 20NAR1-An Alternative NARS Implementation Design. In International Conference on Artificial General Intelligence (pp. 283-291). Springer, Cham.

