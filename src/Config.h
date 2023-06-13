/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef H_CONFIG
#define H_CONFIG

/*-------------------------*/
/* Anticipation parameters */
/*-------------------------*/
//Truth expectation needed for anticipation
#define ANTICIPATION_THRESHOLD_INITIAL 0.501
//Confidence of anticipation failures
#define ANTICIPATION_CONFIDENCE_INITIAL 0.01
//Anticipate for concrete yet unexperienced outcomes derived from generals
#define ANTICIPATE_FOR_NOT_EXISTING_SPECIFIC_TEMPORAL_IMPLICATION true

/*---------------------*/
/* Decision parameters */
/*---------------------*/
//Truth expectation to move on to next component goal in sequence
#define CONDITION_THRESHOLD_INITIAL 0.501
//Desire expectation needed for executions
#define DECISION_THRESHOLD_INITIAL 0.501
//Motor babbling chance
#define MOTOR_BABBLING_CHANCE_INITIAL 0.2
//Decisions above the following threshold will suppress babbling actions
#define MOTOR_BABBLING_SUPPRESSION_THRESHOLD 0.55
//Whether temporal non-procedural implications are allowed to derive subgoals
#define NOP_SUBGOALING true
//Subsumption confidence threshold above which a specific hypothesis inhibits more generals
#define SUBSUMPTION_CONFIDENCE_THRESHOLD 0.05
//Subsumption confidence threshold below which a specific hypothesis inhibits more generals
#define SUBSUMPTION_FREQUENCY_THRESHOLD 0.5
//How long goal events describing bad outcomes are considered in decision making
#define NEG_GOAL_AGE_MAX EVENT_BELIEF_DISTANCE

/*----------------------*/
/* Attention parameters */
/*----------------------*/
//Event selections per cycle for inference
#define BELIEF_EVENT_SELECTIONS 1
//Goal event selections per cycle for inference
#define GOAL_EVENT_SELECTIONS 1
//Event priority decay of events per cycle
#define EVENT_DURABILITY 0.9999
//Concept priority decay of events per cycle
#define CONCEPT_DURABILITY 0.9
//Minimum priority to accept events
#define MIN_PRIORITY 0
//Occurrence time distance in which case event belief is preferred over eternal 
#define EVENT_BELIEF_DISTANCE 20
//Amount of belief concepts to select to be matched to the selected event
#define BELIEF_CONCEPT_MATCH_TARGET 80
//Adaptation speed of the concept priority threshold to meet the match target
#define CONCEPT_THRESHOLD_ADAPTATION 0.000001
//Usage boost for input
#define ETERNAL_INPUT_USAGE_BOOST 1000000
//Unification depth, 2^(n+1)-1, n=2 levels lead to value 7
#define UNIFICATION_DEPTH 31
//Questions concept activation priority
#define QUESTION_PRIMING_INITIAL 0.1

/*---------------------------------*/
/* Temporal compounding parameters */
/*---------------------------------*/
//Maximum length of sequences
#define MAX_SEQUENCE_LEN 2
//Maximum compound op length
#define MAX_COMPOUND_OP_LEN 1
//Max. occurrence time distance between precondition and consequence
#define PRECONDITION_CONSEQUENCE_DISTANCE EVENT_BELIEF_DISTANCE
//Occurrence time distance to now to still correlate an outcome
#define CORRELATE_OUTCOME_RECENCY EVENT_BELIEF_DISTANCE
//Maximum time difference to form sequence between events
#define MAX_SEQUENCE_TIMEDIFF EVENT_BELIEF_DISTANCE
//Allow events which have not been selected to become preconditions
#define ALLOW_NOT_SELECTED_PRECONDITIONS_CONDITIONING false

/*------------------*/
/* Space parameters */
/*------------------*/
//Maximum amount of concepts
#define CONCEPTS_MAX 16384
//Amount of buckets for concept hashmap
#define CONCEPTS_HASHTABLE_BUCKETS CONCEPTS_MAX
//Maximum amount of belief events attention buffer holds
#define CYCLING_BELIEF_EVENTS_MAX 40
//Maximum amount of goal events attention buffer holds
#define CYCLING_GOAL_EVENTS_MAX 400
//Maximum amount of operations which can be registered
#define OPERATIONS_MAX 10
//Maximum amount of arguments an operation can babble
#define OPERATIONS_BABBLE_ARGS_MAX 10
//Maximum size of the stamp in terms of evidental base id's
#define STAMP_SIZE 10
//Maximum Implication table size
#define TABLE_SIZE 20
//Maximum compound term size
#define COMPOUND_TERM_SIZE_MAX 64
//Max. amount of atomic terms, must be <= 2^(sizeof(Atom)*8)
#define ATOMS_MAX 65536
//Amount of buckets for atoms hashmap
#define ATOMS_HASHTABLE_BUCKETS ATOMS_MAX
//The type of an atom
#define Atom unsigned short
//Maximum size of atomic terms in terms of characters
#define ATOMIC_TERM_LEN_MAX 32
//Maximum size of Narsese input in terms of characters
#define NARSESE_LEN_MAX 2148 //ATOMIC_TERM_LEN_MAX * COMPOUND_TERM_SIZE_MAX + 100 for punctuation event marker and TV
//Goal events queue derivation depth layers
#define CYCLING_GOAL_EVENTS_LAYERS 30
//Hashtable bucket size for atom counters in term
#define VAR_INTRO_HASHTABLE_BUCKETS COMPOUND_TERM_SIZE_MAX
//OccurrenceTimeIndex size (large enough to cover all events input and derived within EVENT_BELIEF_DISTANCE from currentTime)
#define OCCURRENCE_TIME_INDEX_SIZE 512

/*------------------*/
/* Truth parameters */
/*------------------*/
//Default frequency for input events
#define NAR_DEFAULT_FREQUENCY  1.0
//Default confidence for input events
#define NAR_DEFAULT_CONFIDENCE 0.9
//Default confidence for analytical premise
#define RELIANCE 0.9
//NAL evidental horizon
#define TRUTH_EVIDENTAL_HORIZON_INITIAL 1.0
//Time distance based projection decay of event truth
#define TRUTH_PROJECTION_DECAY_INITIAL 0.8
//Maximum value for confidence
#define MAX_CONFIDENCE 0.99
//Minimum confidence to accept events
#define MIN_CONFIDENCE 0.01

/*-----------------------*/
/* Derivation parameters */
/*-----------------------*/
//The NAL level of semantic inference
#define SEMANTIC_INFERENCE_NAL_LEVEL 7
//Filter for twice appearing atoms
#define ATOM_APPEARS_TWICE_FILTER true
//Filter for derivations which include nested implications or equivalences
#define NESTED_HOL_STATEMENT_FILTER true
//Filter for inheritance or similarity statement with dependent var
#define INH_OR_SIM_HAS_DEP_VAR_FILTER true
//We don't allow higher-order statements with <A --> A> or <var1 --> var2> components
#define HOL_STATEMENT_COMPONENT_HAS_INVALID_INH_OR_SIM_FILTER true
//Whether a higher-order statement is invalid if it contains a inh or sim without var
#define HOL_COMPONENT_NO_VAR_IS_INVALID_FILTER true
//Whether a higher-order statement is invalid if it contains a inh or sim without atomic term
#define HOL_COMPONENT_NO_ATOMIC_IS_INVALID_FILTER true
//Filter disjunction or conjunction in derivation if not right nested
#define JUNCTION_NOT_RIGHT_NESTED_FILTER true
//Variables introduced in set with more than 1 element filter
#define VARS_IN_MULTI_ELEMENT_SETS_FILTER true
//Filtering sub-statement terms with variables and atoms both like (&, $1, a)
#define TERMS_WITH_VARS_AND_ATOMS_FILTER true

#endif
