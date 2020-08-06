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

volatile bool Stopped = false;
pthread_cond_t start_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t start_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t nar_mutex = PTHREAD_MUTEX_INITIALIZER;

void* Reasoner_Thread_Run(void* timestep_address)
{
    pthread_mutex_lock(&start_mutex);
    pthread_cond_signal(&start_cond);
    pthread_mutex_unlock(&start_mutex);
    long timestep = *((long*) timestep_address);
    assert(timestep >= 0, "Nonsensical timestep for UDPNAR!");
    while(!Stopped)
    {
        pthread_mutex_lock(&nar_mutex);
        NAR_Cycles(1);
        pthread_mutex_unlock(&nar_mutex);
        if(timestep >= 0)
        {
            nanosleep((struct timespec[]){{0, timestep}}, NULL); //POSIX sleep for timestep nanoseconds
        }
    }
    return NULL;
}

void* Receive_Thread_Run(void *sockfd_address)
{
    pthread_mutex_lock(&start_mutex);
    pthread_cond_signal(&start_cond);
    pthread_mutex_unlock(&start_mutex);
    int sockfd = *((int*) sockfd_address);
    for(;;)
    {
        char buffer[NARSESE_LEN_MAX];
        UDP_ReceiveData(sockfd, buffer, NARSESE_LEN_MAX);
        if(Stopped) //avoids problematic buffer states due to socket shutdown, most portable solution!
        {
            break;
        }
        pthread_mutex_lock(&nar_mutex);
        int cmd = Shell_ProcessInput(buffer);
        if(cmd == SHELL_RESET) //reset?
        {
            Shell_NARInit();
        }
        pthread_mutex_unlock(&nar_mutex);
    }
    return NULL;
}

pthread_t thread_reasoner, thread_receiver;
bool Started = false;
int receiver_sockfd; 
void UDPNAR_Start(char *ip, int port, long timestep)
{
    assert(!Stopped, "UDPNAR was already started!");
    Shell_NARInit();
    receiver_sockfd = UDP_INIT_Receiver(ip, port);
    //Create reasoner thread and wait for its creation
    pthread_mutex_lock(&start_mutex);
    pthread_create(&thread_reasoner, NULL, Reasoner_Thread_Run, &timestep);
    pthread_cond_wait(&start_cond, &start_mutex);
    pthread_mutex_unlock(&start_mutex);
    //Create receive thread and wait for its creation
    pthread_mutex_lock(&start_mutex);
    pthread_create(&thread_receiver, NULL, Receive_Thread_Run, &receiver_sockfd);
    pthread_cond_wait(&start_cond, &start_mutex);
    pthread_mutex_unlock(&start_mutex);
    puts("//UDPNAR started!");
    fflush(stdout);
    Started = true;
}

void UDPNAR_Stop()
{
    assert(Started, "UDPNAR not started, call UDPNAR_Start first!");
    Stopped = true;
    shutdown(receiver_sockfd, SHUT_RDWR); //sufficient on Linux to get out of blocking ops on socket, insufficient on Mac
    close(receiver_sockfd); //sufficient on Mac to get out of blocking ops on socket, insufficient on Linux (hence, use both!)
    pthread_join(thread_reasoner, NULL);
    pthread_join(thread_receiver, NULL);
    Stats_Print(currentTime);
}
