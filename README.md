# ANSNA
Adaptive Neuro-Symbolic Network Agent

Goal
----
This project is trying to combine my most valuable insights about Dr. Pei Wang's NARS (Non-Axiomatic Reasoning System), Jeffrey Hawkins HTM (Hierarchical Temporal Memory), Tony Lofthouse's ALANN (Adaptive Logic and Neural Network) and my own projects of the last decade, for creating an autonomous sensorimotor agent that starts with zero knowledge and organizes its experience into conceptual units in such an efficient way that it can be applied for all kind of autonomous systems.

Closing the loop
----------------
Especially since the great successes of DL (Deep Learning), albeit mostly used to build passive sensory stimuli classifiers, symbolic systems suffer from bad reputation. Also justified, as many of them are rule based systems without any learning capacity whatsoever, mostly only following deductive ways of thinking. But there are exceptions, systems like NARS or OpenCog that can learn useful knowledge from their experience, by a combination of inference and other data mining processes:
- OpenNARS: https://www.youtube.com/watch?v=hAO1zRj2z9A
- OpenNARS: https://www.youtube.com/watch?v=GEmcT4Nk-78
- OpenCog: https://www.youtube.com/watch?v=X8C7nhIULZs

These current proto-AGI systems "close the loop", they go from perception to conceptual knowledge, and then to action. Definitely not by keeping track of and simply using the action of highest utility for a situation, but because the consequences of applying the action, in the current context, are both understood and desired by the AGI system.

Learning from the past
----------------------
The field of AGI is full of misconceptions. In fact, the issues of logic-based systems was never that they use logic, but what kind of logic they use. Also it was never about that they use symbolic representations. In the moment of realizing that a cognitive logic like Non-Axiomatic Logic (https://www.worldscientific.com/worldscibooks/10.1142/8665) or Probabilistic Logic Networks (https://www.springer.com/us/book/9780387768717) has no issue whatseover to encode and reason about the experienced values of pixels, involving activation values, as one would assign to an ANN receiving that same input, it becomes clear that the knowledge representation these logics provide might be at best too mighty or inefficient, but in no way a restriction of any kind. A fundamental restriction of the traditional rule based systems was simply that they didn't have a way to measure the "truth" of a hypothesis in a statistical manner, a masurement of evidental support or some kind of higher-order probability. And how can we expect a system to learn anything useful from the environment without an ability to exploit correlations beteween events? ANSNA takes the position, that everything can be learned and conceptualized based on some kind of, often hierarchical, composition of observed spatial and temporal correlations (everything is experience-grounded), and how much of them can be constructed, explored and evaluated is a matter of resources the system can invest.

AIKR
----
Same as NARS as proposed by Dr. Pei Wang, ANSNA takes the Assumption of Insufficient Knowledge and Resources (AIKR) as fundamental working assumption. Having understood this assumption, there is simply no rational way in believing that AGI could be built without obeying this principle. When resources wouldn't be an issue that need to be addressed, for deciding in which of the, seemingly, infinite directions, to think, in every moment in time, my precious reader, then please take your AIXI (https://en.wikipedia.org/wiki/AIXI) and go home. Or consider, that in such an universe, human attention for resource allocation within the brain would be superfluous, we would never forget, AGI would likely be everywhere, all the worms would have it, it's really not difficult to evolve a single equation, if following it would be anywhere near feasible and fruitful. 
For more on the topic, read: "Insufficient Knowledge and Resources — A Biological Constraint and Its Functional Implications": https://pdfs.semanticscholar.org/aa7c/5b8b49eb132643242987cb5f4c45ededb2be.pdf

Attention and structures to attend
----------------------------------
What requirements do the structures need to fullfill that we manipulate in our mind for all kind of purposes? What exactly happens in our mind if we think about our blue planet and jump to the oceans? Is everything we think about something we directly observe, simply some kind of experience-replay? Or do we compose structures we have never seen before, based on what seems to make sense to compose in a given context? NARS, ALANN, and ANSNA take the latter position, also taking creativity as an essential property of AGI. But how are these structures represented? As some kind of composition of ID's (NARS compound terms), as bit vectors (HTM SDR's), as some kind of implicit, potentially temporally unstable, transient, state in firing patterns (spiking neural networks)? As a subset of collections of weights in an old-school deep neural network? ANSNA takes the position that SDR's are a highly efficient way to encode mental compositions. Some of my initial work to support sensorimotor-relevant subsets, of the cognitive logic NAL, developed bei Dr. Pei Wang, without using explicit terms or symbols, already suggests that: https://bit.ly/2NfgHc7
https://bit.ly/2OwGdGl

Given certain experienced patterns exist within a mind, according to what criteria are they considered, used, de-priorized, or even completely forgotten? The ALANN model by Tony Lofthouse provides the potentially most promising answer to that question. Having studied attention mechanisms for NARS, AERA, various FARG architectures (https://cogsci.indiana.edu/book.html), and others as discussed in Helgi Páll Helgason's Ph.D. thesis (see https://en.ru.is/media/td/Helgi_Pall_Helgason_PhD_CS_HR.pdf), and extensively tested a prototype of the ALANN system, it is likely most complete solution to the resource allocation problem in an artificial mind.

ANSNA in a nutshell
-------------------
- Overcoming symbolic limitations (space usage, inefficiency in matching) by the use of SDR encodings: SDR-based representation of experienced and inferred structures.
- Concept-centric SDR-based memory structure to efficiently capture structure of the world.
- Cognitive logic applied on SDR's rather than "symbols", restricted to NAL7/8: sequences and implications to exploit and use correlations in inputs.
- ALANN control system.
- Intended to be much simpler and orders of magnitudes more efficient for sensorimotor learning than OpenNARS could be. OpenNARS was designed to also work with high-level user input, for which case the compound term/symbol encodings for output etc., and tons of additional inference rule, that slow the system down, for such purposes, are necessary.

ANSNA the name
--------------
Adaptive - ALANN control model to obey AIKR and to solve the mental resource allocation problem.

Neuro-Symbolic - "Sub-symbolic" SDR encodings, yet all advantages of cognitive logic.

Network - Nodes learning conceptual structure while having several neural network properties, such as activation spreading.

Agent - Goal to "close the loop", to learn conceptual knowledge from sensor data and to act according to own motivations.

So what is ANSNA
----------------
Neural network, or reasoning system? I don't know, maybe something in-between, or both, must I decide? :)
Hopefully one day a thinking machine that learns everything from sensory data and interacts with the environment.

What ANSNA, NARS and ALANN have in common, important properties of AGI
----------------------------------------------------------------------
1. Obeying AIKR.
0. Experience-grounded rather than model-theoretic.
1. Attention Mechanism and important system-eigendynamics as a source of creativity.
2. Lifelong online learning.
4. NAL-based evidence measurement.
3. Conceptual representations of patterns in the experience.
5. Real-time operation: Ability to deal with events that come in at any moment.
6. Knowledge representation that has both symbolic and sub-symbolic, or neuro-symbolic, aspects.
