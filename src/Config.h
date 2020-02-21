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

#ifndef CONFIG_H
#define CONFIG_H

/*-------------------------*/
/* Anticipation parameters */
/*-------------------------*/
//Truth expectation needed for anticipation
#define ANTICIPATION_THRESHOLD_INITIAL 0.501
//Confidence of anticipation failures
#define ANTICIPATION_CONFIDENCE_INITIAL 0.005

/*-----------------------*/
/* Subgoaling parameters */
/*-----------------------*/
//Truth expectation threshold for subgoals to be propagated
#define PROPAGATION_THRESHOLD_INITIAL 0.501
//How many propagation iterations happen per cycle
#define PROPAGATION_ITERATIONS 5

/*---------------------*/
/* Decision parameters */
/*---------------------*/
//Truth expectation needed for executions
#define DECISION_THRESHOLD_INITIAL 0.501
//Motor babbling chance
#define MOTOR_BABBLING_CHANCE_INITIAL 0.2
//Decisions above the following threshold will suppress babbling actions
#define MOTOR_BABBLING_SUPPRESSION_THRESHOLD 0.6

/*----------------------*/
/* Attention parameters */
/*----------------------*/
//Amount of threads
#define THREADS 1
//Event selections per cycle for inference
#define EVENT_SELECTIONS 1
//Event priority decay of events per cycle
#define EVENT_DURABILITY 1.0
//Additional event priority decay of an event which was selected
#define EVENT_DURABILITY_ON_USAGE 0.0
//Concept priority decay of events per cycle
#define CONCEPT_DURABILITY 0.9
//Minimum confidence to accept events
#define MIN_CONFIDENCE 0.01
//Minimum priority to accept events
#define MIN_PRIORITY 0
//Occurrence time distance in which case event belief is preferred over eternal 
#define EVENT_BELIEF_DISTANCE 20
//Amount of belief concepts to select to be matched to the selected event
#define BELIEF_CONCEPT_MATCH_TARGET 80
//Adaptation speed of the concept priority threshold to meet the match target
#define CONCEPT_THRESHOLD_ADAPTATION 0.000001

/*------------------*/
/* Space parameters */
/*------------------*/
//Maximum amount of concepts
#define CONCEPTS_MAX 16384
//Maximum amount of events attention buffer holds
#define CYCLING_EVENTS_MAX 20
//Maximum amount of operations which can be registered
#define OPERATIONS_MAX 10
//Maximum size of the stamp in terms of evidental base id's
#define STAMP_SIZE 20
//Maximum event FIFO size
#define FIFO_SIZE 20
//Maximum Implication table size
#define TABLE_SIZE 20
//Maximum length of sequences
#define MAX_SEQUENCE_LEN 3
//Maximum compound term size
#define COMPOUND_TERM_SIZE_MAX 64
//Max. amount of atomic terms, must be <= 2^(sizeof(Atom)*8)
#define TERMS_MAX 256
//The type of an atom
#define Atom unsigned char
//Maximum size of atomic terms in terms of characters
#define ATOMIC_TERM_LEN_MAX 30
//Maximum size of Narsese input in terms of characters
#define NARSESE_LEN_MAX 1000

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

#endif
