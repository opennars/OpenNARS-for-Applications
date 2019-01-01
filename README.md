# ANSNA
Adaptive Neuro-Symbolic Network Agent

ANSNA is an adaptive sensorimotor system that makes use of events, each holding a Sparse Distributed Representation (SDR). ANSNA is able to compose and decompose them into new events using union, tuple, intersection and union operations, and can learn predictive co-occurrence statistics in terms of which sets of events predict which others, using principles of Non-Axiomatic Logic. All events are mapped to a container (concept) with the closest SDR to itself, and both input and derived ones come with an attention value attached, where only high-priority events will be processed often, and the lowest ones forgotten at full capacity, to work under the available resource supply. Some of these events are beliefs, and lead to the derivation of predicted events, or the strengthening of learned predictive implications, and some are goals, and lead to the derivation of sub-goals. Some of these derived goals have a SDR that triggers an operation. In summary, the SDR encoding allows to effectively match aspects of a similar but in certain aspects different new situation to already known ones, and the learning of the predictive co-occurrence statistics lets the system learn new ways to achieve or predict certain outcomes, as to become more capable to reach its goals over time.

Goal
----
This project is trying to combine my most valuable insights about Dr. Pei Wang's NARS (Non-Axiomatic Reasoning System), Jeffrey Hawkins HTM (Hierarchical Temporal Memory), Tony Lofthouse's ALANN (Adaptive Logic and Neural Network), Rod Rinkus's Sparsey, Pentti Kanerva's SDM (Sparse distributed memory), and my own projects of the last decade, for creating an autonomous sensorimotor agent that starts with zero knowledge and organizes its experience into conceptual units in such an efficient way that it can directly be applied for autonomous systems with rich sensory data. (Wiki: https://github.com/patham9/ANSNA/wiki)

Closing the loop
----------------
Especially since the great successes of DL (Deep Learning), albeit mostly used to build passive sensory stimuli classifiers, symbolic systems suffer from bad reputation. Also justified, as many of them are rule based systems without any learning capacity whatsoever, mostly only following deductive ways of reasoning. But there are exceptions, systems like NARS or OpenCog that can learn novel knowledge from their experience, by a combination of inference and other data mining processes:
- OpenNARS: https://www.youtube.com/watch?v=hAO1zRj2z9A
- OpenNARS: https://www.youtube.com/watch?v=GEmcT4Nk-78
- OpenCog: https://www.youtube.com/watch?v=X8C7nhIULZs

These current proto-AGI systems "close the loop", in the sense that they realize everything, from perception to the formation and usage of conceptual knowledge, and ultimatively, also decision making. Not by keeping track of, and simply using, the action of highest utility for a situation, but because the consequences of applying the action, in the current context, are both understood and desired by the AGI system.

Learning from the past
----------------------
The field of AGI is full of misconceptions. In fact, the issues of logic-based systems was never that they use logic, but what kind of logic they use. Also it was never about that they use symbolic representations. For instance, is easy to see that a cognitive logic, such as Non-Axiomatic Logic (https://www.worldscientific.com/worldscibooks/10.1142/8665) or Probabilistic Logic Networks (https://www.springer.com/us/book/9780387768717) has no issue to encode and reason about the experienced brightness values of pixels, with different granularity. The knowledge representation these logics provide might be at best too mighty or inefficient to use, but in no way is that a fundamental restriction of any kind. A fundamental restriction of the traditional rule based systems was simply that they didn't have a way to measure the "truth" of a hypothesis in a statistical manner, a masurement of evidental support or some kind of higher-order probability, measurements these cognitive logics support. And how can we expect a system to learn anything from the environment without an ability to exploit correlations beteween events? ANSNA takes the position, that everything can be learned and conceptualized based on some kind of, often hierarchical, composition of observed spatial and temporal correlations (everything is grounded into sensorimotor-experience), and how much of them can be constructed, explored and evaluated is a matter of resources the system can invest. 

AIKR
----
Same as NARS, proposed by Dr. Pei Wang, ANSNA takes the Assumption of Insufficient Knowledge and Resources (AIKR) as fundamental working assumption. Having understood this assumption, there is simply no rational way in believing that AGI could be built without obeying this principle. When resources wouldn't be an issue that need to be addressed, for deciding in which of the, seemingly, infinite directions, to reason, in every moment in time, my precious reader, then please take your AIXI (https://en.wikipedia.org/wiki/AIXI) and go home. Or consider, that in such an universe, human attention for resource allocation within the brain would be superfluous, we would never forget, AGI would likely be everywhere, all the worms would have it, it's really not difficult to evolve a single equation, if following it would be anywhere near feasible and fruitful. 
For more on the topic, read: "Insufficient Knowledge and Resources — A Biological Constraint and Its Functional Implications": https://pdfs.semanticscholar.org/aa7c/5b8b49eb132643242987cb5f4c45ededb2be.pdf

Attention and structures to attend
----------------------------------
What requirements do the structures need to fullfill that we manipulate in our mind for all kind of purposes? What exactly happens in our mind if we think about our blue planet and jump to the oceans? Is everything we think about something we directly observed, simply some kind of experience-replay? Or do we compose and manipulate structures in our mind, structures representing partial aspects of a situation, and structures we have potentially never seen before? NARS, ALANN, and ANSNA try to realize the latter. But how are these structures represented? As some kind of composition of ID's (NARS compound terms), as bit vectors (HTM SDR's), as some kind of implicit, potentially temporally unstable, transient, state in firing patterns (spiking neural networks)? As a subset of collections of weights in an old-school deep neural network? ANSNA takes the position that SDR's are a highly efficient way to encode mental compositions.

Given certain experienced patterns exist within a mind, according to what criteria are they considered, used, de-priorized, or even completely forgotten? The ALANN model by Tony Lofthouse provides a very promising answer to that question. Having studied attention mechanisms for NARS, AERA, various FARG architectures (https://cogsci.indiana.edu/book.html), and others as discussed in Helgi Páll Helgason's Ph.D. thesis (see https://en.ru.is/media/td/Helgi_Pall_Helgason_PhD_CS_HR.pdf), and extensively tested a prototype of the ALANN system, I am convinced that it provides a simple and yet almost complete solution to the resource allocation problem in an artificial mind.

ANSNA in a nutshell
-------------------
- Overcoming symbolic limitations (space usage, inefficiency in matching and processing) by the use of SDR encodings: SDR-based representation of experienced and inferred structures.
- Concept-centric memory to efficiently capture structure of the world.
- Cognitive logic applied on SDR's rather than "symbols", restricted to NAL7/8 plus a SDR-based realization of NAL6: sequences and implications to exploit and use correlations in inputs, variable bits for learning general relationships. Inheritance and similarity naturally arise from symmetric and asymmetric SDR overlap evaluations.
- ALANN control system, inspired by spiking neural networks.
- Specialized to large-scale sensorimotor learning, different to OpenNARS which was designed to accept high-level user input: ANSNA comes with a minimal but essential subset of NAL6, NAL7 and NAL8, allowing it to learn predictive hypotheses, general procedure knowledge and categories using SDR encodings. 
- SDR's instead of tree-based compound terms, allow to attach dozens of sensors values to inputs without complicating their processing.

ANSNA the name
--------------
Adaptive - ALANN control model for resource allocation under AIKR.

Neuro-Symbolic - "Sub-symbolic" SDR encodings, while utilizing cognitive logic (NAL).

Network -  Learning conceptual representations while having several neural network properties, such as inference/spike based activation spreading and predictive links between concepts.

Agent - Goal to "close the loop", to learn conceptual knowledge from sensor data and to act according to own motivations.

So what is ANSNA
----------------
Some kind of spiking neural network, or a reasoning system? That's a matter of interpretation, but it tries to combine principles from both paradigms. Hopefully one day it is a system that learns general relationships from raw sensory data and interacts with the environment.

What ANSNA, NARS and ALANN have in common
----------------------------------------------------------------------
1. Obeying AIKR.
0. Experience-grounded rather than model-theoretic semantics.
1. Attention Mechanism and system-eigendynamics as source for creativity.
2. Lifelong online learning.
4. NAL-based evidence measurement.
3. Conceptual representations of patterns in the experience.
5. Real-time operation: Ability to deal with events that come in at any moment.
6. Knowledge representation that has both symbolic and sub-symbolic, or neuro-symbolic, aspects.
