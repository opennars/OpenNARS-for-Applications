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
volatile bool reasoner_thread_started = false, receive_thread_started = false;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void* Reasoner_Thread_Run(void* timestep_address)
{
    long timestep = *((long*) timestep_address);
    reasoner_thread_started = true;
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

void* Receive_Thread_Run(void *sockfd_address)
{
    int sockfd = *((int*) sockfd_address);
    receive_thread_started = true;
    for(;;)
    {
        char buffer[NARSESE_LEN_MAX];
        UDP_ReceiveData(sockfd, buffer, NARSESE_LEN_MAX);
        pthread_mutex_lock(&mutex);
        NAR_AddInputNarsese(buffer);
        pthread_mutex_unlock(&mutex);
    }
    return NULL;
}

pthread_t thread_reasoner, thread_receiver;
bool started = false;
void UDPNAR_Start(char *ip, int port, long timestep)
{
    NAR_INIT();
    assert(!Stopped, "UDPNAR was already started!");
    int sockfd = UDP_INIT_Receiver(ip, port);
    pthread_create(&thread_reasoner, NULL, Reasoner_Thread_Run, &timestep);
    pthread_create(&thread_receiver, NULL, Receive_Thread_Run, &sockfd);
    while(!receive_thread_started || !reasoner_thread_started); //busy wait :)
    puts("//UDPNAR started!");
    started = true;
}

void UDPNAR_Stop()
{
    assert(started, "UDPNAR not started, call UDPNAR_Start first!");
    Stopped = true;
    pthread_cancel(thread_receiver); //reasoner thread doesn't block
    pthread_join(thread_reasoner, NULL);
    pthread_join(thread_receiver, NULL);
}
