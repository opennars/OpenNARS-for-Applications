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

#include "UDPNAR.h"

bool Stopped = false;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
void* Reasoner_Thread(void* timestep_address)
{
	long timestep = *((long*) timestep_address);
	assert(timestep >= 0, "Nonsensical timestep for UDPNAR!");
	while(!Stopped)
	{
		pthread_mutex_lock(&mutex);
		NAR_Cycles(1);
		pthread_mutex_unlock(&mutex);
		if(timestep >= 0)
		{
			nanosleep((struct timespec[]){{0, timestep}}, NULL); //POSIX sleep for timestep nanoseconds
		}
	}
	return NULL;
}

void* Receive_Thread(void *sockfd_and_port)
{
	int sockfd = ((int*) sockfd_and_port)[0];
	int port = ((int*) sockfd_and_port)[1];
	for(;;)
	{
		char buffer[NARSESE_LEN_MAX];
		UDP_GetData(sockfd, buffer, NARSESE_LEN_MAX);
		Term term;
		Truth tv;
		char punctuation;
		bool isEvent;
		Narsese_Sentence(buffer, &term, &punctuation, &isEvent, &tv);	
#if STAGE==2
		term = RuleTable_Reduce(term, false);
#endif
		pthread_mutex_lock(&mutex);
		NAR_AddInput(term, punctuation, tv, isEvent);
		pthread_mutex_unlock(&mutex);
	}
	return NULL;
}

void UDPNAR_Start(char *ip, int port, long timestep)
{
	int sockfd_and_port[] = { UDP_INIT(ip, port), port };
	pthread_t thread_id_reasoner, thread_id_receiver;
    pthread_create(&thread_id_reasoner, NULL, Reasoner_Thread, &timestep);
    pthread_create(&thread_id_receiver, NULL, Receive_Thread, sockfd_and_port);
    puts("UDPNAR started, for cancellation press any key!");
	getchar();
	Stopped = true;
	pthread_cancel(thread_id_receiver); //reasoner thread doesn't block
	pthread_join(thread_id_reasoner, NULL);
	pthread_join(thread_id_receiver, NULL);
}
