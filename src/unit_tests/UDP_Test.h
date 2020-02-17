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

#include "./../NetworkNAR/UDP.h"
#include <pthread.h>

void *Receiver_Test_Thread_Run(void *sockfd_receiver_address)
{
    int sockfd_receiver = *((int*) sockfd_receiver_address);
    int receive_size = 1024;
    char receive_data[receive_size];
    UDP_ReceiveData(sockfd_receiver, receive_data, receive_size);
    assert(!strcmp(receive_data, "<(a &/ ^left) =/> g>."), "We didn't receive what we sent!");
    return NULL;
}

void UDP_Test()
{
    puts(">>UDP test start");
    char *ip = "127.0.0.1";
    int port = 50000;
    int sockfd_receiver = UDP_INIT_Receiver(ip, port);
    pthread_t thread_receiver;
    pthread_create(&thread_receiver, NULL, Receiver_Test_Thread_Run, &sockfd_receiver);
    nanosleep((struct timespec[]){{0, 10000000L}}, NULL); //wait for 10ms
    int sockfd_sender = UDP_INIT_Sender();
    char *send_data = "<(a &/ ^left) =/> g>.";
    UDP_SendData(sockfd_sender, ip, port, send_data, strlen(send_data)+1);
    pthread_join(thread_receiver, NULL);
    puts(">>UDP test successul");
}
