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

#include "./../NetworkNAR/UDPNAR.h"

bool NAR_UDPNAR_Test_op_left_executed = false;
Feedback NAR_UDPNAR_Test_op_left()
{
    NAR_UDPNAR_Test_op_left_executed = true;
    return (Feedback) {0};
}

void NAR_UDPNAR_Test()
{
    puts(">>UDPNAR test start");
    char *ip = "127.0.0.1";
    int port = 50001;
    long timestep = 10000000L; //10ms
    UDPNAR_Start(ip, port, timestep);
    NAR_AddOperation("^left", NAR_UDPNAR_Test_op_left);
    int sockfd_sender = UDP_INIT_Sender();
    char *send_data1 = "<(a &/ ^left) =/> g>.";
    UDP_SendData(sockfd_sender, ip, port, send_data1, strlen(send_data1)+1);
    char *send_data2 = "a. :|:";
    UDP_SendData(sockfd_sender, ip, port, send_data2, strlen(send_data2)+1);
    char *send_data3 = "g! :|:";
    UDP_SendData(sockfd_sender, ip, port, send_data3, strlen(send_data3)+1);
    nanosleep((struct timespec[]){{0, timestep}}, NULL); //wait another timestep
    assert(NAR_UDPNAR_Test_op_left_executed, "UDPNAR operation wasn't executed!!");
    UDPNAR_Stop();
    puts(">>UDPNAR test successul");
}
