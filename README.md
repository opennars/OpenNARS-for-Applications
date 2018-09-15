# ANSNA
Adaptive Neuro-Symbolic Network Agent

Goal
----
This project is trying to combine my most valuable insights about Pei Wang's NARS (Non-Axiomatic Reasoning System), Jeffrey Hawkins HTM (Hierarchical Temporal Memory), Tony Lofthouse's ALANN (Adaptive Logic and Neural Network) and my own projects of the last decade, for creating an autonomous sensorimotor agent that starts with zero knowledge and organizes its experience into conceptual units in such an efficient way that it can be applied for autonomous systems with zero initial knowledge.

Closing the loop
----------------
Especially since the great successes of DL (Deep Learning), and although DL technology is mostly only be used to build passive sensory stimuli classifiers, symbolic systems suffer from bad reputation. Also justified, as many of them are rule based systems without any learning capacity whatsoever, mostly only following deductive ways of thinking. But there are exceptions, systems like NARS or OpenCog that can learn useful knowledge from their experience, by a combination of inference and other data mining processes:
- OpenNARS: https://www.youtube.com/watch?v=hAO1zRj2z9A
- OpenNARS: https://www.youtube.com/watch?v=GEmcT4Nk-78
- OpenCog: https://www.youtube.com/watch?v=X8C7nhIULZs

These current proto-AGI systems "close the loop", they go from perception to conceptual knowledge, and then to action. Definitely not by keeping track of and simply using the action of highest utility for a situation, but because the consequences of applying the action, in the current context, are both understood and desired by the AGI system.

Learning from the past
----------------------
The field of AGI is full of misconceptions. In fact, the issues of logic-based systems was never that they used logic, but what kind of logic they used. Also it was never about that they used symbolic representations. In the moment of realizing that a cognitive logic like Non-Axiomatic Logic (https://www.worldscientific.com/worldscibooks/10.1142/8665) or Probabilistic Logic Networks (https://www.springer.com/us/book/9780387768717) has no issue whatseover to encode and reason about the experienced values of pixels, involving activation values, as one would assign to an ANN receiving that same input, it becomes clear that the knowledge representation these logics provide might be at best too mighty, but in no way a restriction of any kind. A key issue of the traditional rule based systems was simply that they didn't have a way to measure the "truth" of a hypothesis in a statistical manner, a masurement of evidental support or some kind of higher-order probability. And how can we expect a system to learn anything useful from the environment without an ability to exploit correlations beteween events? ANSNA takes the position, that everything can be learned and conceptualized based on some kind of, often hierarchical, composition of observed spatial and temporal correlations (everything is experience-grounded), and how much of them can be constructed, explored and evaluated is a matter of resources the system can invest.

AIKR
----
Same as NARS, ANSNA takes the Assumption of Insufficient Knowledge and Resources (AIKR) as fundamental working assumption. Having understood this assumption, there is simply no rational way in believing that AGI could be built without obeying this principle. When resources wouldn't be an issue that need to be addressed, for deciding in which of the, seemingly, infinite directions, to think, in every moment in time, my precious reader, then please take your AIXI (https://en.wikipedia.org/wiki/AIXI) and go home. In such an universe, human attention for resource allocation within the brain would be superfluous, and AGI would likely be everywhere, all the worms would have it, it's really not difficult to evolve a single equation, if following it would be anywhere near feasible. 
For more on the topic, read: "Insufficient Knowledge and Resources — A Biological Constraint and Its Functional Implications": https://pdfs.semanticscholar.org/aa7c/5b8b49eb132643242987cb5f4c45ededb2be.pdf

Attention and structures to attend
----------------------------------
What requirements do the structures need to fullfill that we manipulate in our mind for all kind of purposes? What exactly happens in our mind if we think about our blue planet and jump to the oceans? Is everything we think about something we directly observe, simply some kind of experience-replay? Or do we compose structures we have never seen before, based on what seems to make sense to compose in a given context? NARS, ALANN, and ANSNA take the latter position, also taking creativity as an essential property of AGI. But how are these structures represented? As some kind of composition of ID's (NARS compound terms), as bit vectors (HTM SDR's), as some kind of implicit, potentially temporally unstable, transient, state in firing patterns (spiking neural networks)? As a subset of collections of weights in an old-school deep neural network? ANSNA takes the position that SDR's might be the most efficient way to encode mental compositions, with some initial work to support sensorimotor-relevant subsets of cognitive logic like NAL without using explicit terms or symbols: http://aleph.sagemath.org/?z=eJy1Vttu4zYQfTfgfxg4QCoCzG1RFIVR76Jo-1CgXRR13oyFQVF0zMK6LEkl3W367x1eRFGxpXVaNA-xNJw5Z3g4M6Ism1oZUKwq6nI-m88udlJpQ6FkDRihSg2mhgdRCcWMKGD94-9faUDv8ISL3mtXK2hqIyoj2QHEQZT4CI2SaKge5jPrtUXQ1V9_hxctPwtYwZvbr7-dzxpV20RkXW3v0Hh7fXs3nxVih9TmHr2zipWCLOczwL-HQ50jSQfpjXIH1gdkFReCu_1TwrSqX9lY1w9-eWhD8s2dBfOSXPufjHw3yFActIBbt-ncMqLXg8jitkiAHmFFldc1ppozLbkVEurG6ovYXkgtjNPYtM1BgKh4XTgRrSDo_qusWp0xmneCBJ5Nyf7MbinbyA9XOf4jDkyOJBjA1sKcgrIgzxZkEkOLj-0Wcy9Rtt_wpzV-F4lXUHAbSiIjSdRWVo8YGd-v8V0oLaxTSO_eSnAiwZh5Es24Jc9yQpzG7-snMHsB77__xRcpIvxQV7xVytbmKtbWorcuAjOPlnHuPoqmQiLCGlPCUxMpR2frGHR4H8fvImiqAziCn8vmILnTOuVIzB2N7E3jTEncCbKLtQUROBIM3yelilDOdL9XQu_rQ-Ea95u-TO1i1jBl8IXu2sMBf8kSLjyQXaBgzcBZBXv2iBQtLpS1EnAHuTSaugNEGbTbqtTwtBdoUs7ufaDeuTeLBwxD7brw29Qt50JrW2NtmW1CLljVlyGd6QInNy_Cpnw944UbebBAlbitMrfXJSxoSOW6yggMzqDL8S0M1exbAEnuPzUjleJVtksx4KePLTu4ivhCRDfIvSWnrMfA4jZMVnrd5uYcJNQlGo46khGX3nzG0mpltkbz1JJbC7beH1tm7S-a0H4lnLYWZUGBRYN1XgCWa7Rkl8_U5bVCx4CYxGMhab8xWz2d87sFPb31AEASwlch5McIMcEhUD6CEOKdCiiv12cwQgJyQnDTK-BDJgS4mRbAx0_tfxIgPwKI2SEOixt5Bz4-VnwcgdPxfaG4s24rV3ZHaMnAjng42ryYLwflkZ5Ydau3ECir1P-IKJ2nnoCM4Jyx9QiAk9iqeYXXBryJFcIwvne3rKwQReu2TEAb0SwhcGCPwPOVbQvb1X3EFsfYE1MFlhXe_9qqoO6hwqWuyeVuTLouhhxfr-LdJD6cIKA9FjnJFXc-wTS4QnRj5jz6Dp6cZk_P7v9JIGEgxELGUsu3SnB7CVydPiw_AxgZDr507uVvvCEAJf1y6fqF5q4guo-O_Qp-Qi4lH0Wx9BXYfz1w3AQccmaKvq3-ZYa-oyn7jxny9IvCF2Fg8uHE5P0E8A9f3BYCUE56Ac7QIbi8Ro5TqnDaaUMot-LE4XeWRsNcYGqSdq2xHB1GMEAj5-X5uiMdKmHvoPPZPzTNvkM=&lang=sage
A big question that remains unanswered is, according to what criteria are patterns considered, used, de-priorized, or even completely forgotten? The ALANN model by Tony Lofthouse provides the potentially most promising answer to that question. Having studied attention mechanisms for NARS, AERA, various FARG architectures (https://cogsci.indiana.edu/book.html), and others as discussed in Helgi Páll Helgason's Ph.D. thesis (see https://en.ru.is/media/td/Helgi_Pall_Helgason_PhD_CS_HR.pdf), and extensively tested a prototype of the ALANN system, it is likely everything necessary to solve the resource allocation problem in an artificial mind.

ANSNA in a nutshell
-------------------
- Concept-centric SDR-based memory structure
- SDR-based representation of experienced and inferred structures.
- Cognitive logic applied on SDR's, restricted to NAL7/8: sequences and implications to exploit and use correlations in inputs
- ALANN control system


What ANSNA, NARS and ALANN have in common
-----------------------------------------
1. Obeying AIKR.
0. Experience-grounded rather than model-theoretic.
1. Attention Mechanism and important system-eigendynamics as a source of creativity.
2. Lifelong online learning.
4. NAL-based evidence measurement.
3. Conceptual representations of patterns in the experience.
5. Real-time operation: Ability to deal with events that come in at any moment.
6. Knowledge representation that has both symbolic and sub-symbolic, or neuro-symbolic, aspects.





