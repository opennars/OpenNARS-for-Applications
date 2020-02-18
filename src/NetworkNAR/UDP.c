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

#include "UDP.h"

int UDP_INIT_Receiver(char *ip, int port)
{
    int sockfd;
    struct sockaddr_in address_me = {0};
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    address_me.sin_family = AF_INET;
    address_me.sin_port = htons(port);
    address_me.sin_addr.s_addr = inet_addr(ip);
    bind(sockfd, (struct sockaddr*)&address_me, sizeof(address_me));
    return sockfd;
}

int UDP_INIT_Sender()
{
    return socket(PF_INET, SOCK_DGRAM, 0);
}

void UDP_ReceiveData(int sockfd, char *buffer, int buffersize)
{
    struct sockaddr_in address_other;
    socklen_t addr_size = sizeof(address_other);
    recvfrom(sockfd, buffer, buffersize, 0, (struct sockaddr*)& address_other, &addr_size);
    IN_DEBUG( printf("//UDP Data received: %s\n", buffer); )
}

void UDP_SendData(int sockfd, char *ip, int port, char *buffer, int buffersize)
{
    struct sockaddr_in address_destination = {0};
    address_destination.sin_family = AF_INET;
    address_destination.sin_port = htons(port);
    address_destination.sin_addr.s_addr = inet_addr(ip);
    sendto(sockfd, buffer, buffersize, 0, (struct sockaddr*)&address_destination, sizeof(address_destination));
}
