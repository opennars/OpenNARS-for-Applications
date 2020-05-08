"""
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
 * """

import thread 
from subprocess import Popen, PIPE, STDOUT
import subprocess
import re,os
import socket
import sys
import time
import itertools

server = "irc.freenode.net"
channel = "#nars"
botnick = "yan42"
Narsese_Filter=["^","Answer:"]
process_start = ["./../NAR","shell"]

proc = subprocess.Popen(process_start, stdin=subprocess.PIPE, stdout=subprocess.PIPE) 
irc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
irc.connect((server, 6667))
irc.send("USER "+ botnick +" " + botnick +" "+ botnick +" :Hallo!\r\n")
irc.send("NICK "+ botnick +"\r\n")
irc.send("PRIVMSG nickserv :iNOOPE\r\n")
irc.send("JOIN "+ channel +"\r\n") #fails on euirc, works in all others, euirc is not completely RFC 2810 compatible maybe?
print "connected"
print "NARS instance created"
print "Narsese Filter applied: '"+str(Narsese_Filter)+"'"

def receive_thread(a):
    while True:
       msg=proc.stdout.readline()
       if msg!=None and msg!="" and True in [u in msg for u in Narsese_Filter]:
          print "NAR output: "+msg
          irc.send("PRIVMSG "+ channel +" : "+msg+"\r\n")   
thread.start_new_thread(receive_thread,(1,))

while True:
    try:
        text=irc.recv(2040)
        if "PING" in text:
            STR='PONG :' + text.split("PING :")[1].split("\n")[0] + '\r\n';
            irc.send(STR)
        else:
            if "VERSION" in text:
                irc.send("JOIN "+ channel +"\r\n") #join when version private message comes :D
            else:
                SPL=text.split(":")
                TEXT=":".join(SPL[2:len(SPL)])
                if TEXT.replace(" ","").replace("\n","").replace("\r","")=="":
		    continue
                if TEXT.startswith("*reset"):
                    proc.stdin.write("*reset\n")
                if TEXT.startswith("yan: ") or TEXT.startswith("(") or TEXT.startswith("<") or TEXT.startswith("100"):
		    if TEXT.startswith("yan: "):
                        TEXT = TEXT.split("yan: ")[1]
                    print "NAR input: "+TEXT
                    try:
                        proc.stdin.write(TEXT+"\n")
                    except:
                        None
    except:
        print "exception"
None
