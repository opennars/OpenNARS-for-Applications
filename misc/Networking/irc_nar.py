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

import sys
version = sys.version_info[0]
if version == 2:
    import thread
else:
    import _thread as thread
from subprocess import Popen, PIPE, STDOUT
import subprocess
import re,os
import socket
import time
import itertools

server = "irc.libera.chat"
channel = "#nars"
botnick = "nars42"
Narsese_Filter=[" executed with args","Answer:"]


def encode(st):
    if version == 2:
        return st
    else:
        return st.encode("utf-8")

def decode(st):
    if version == 2:
        return st
    else:
        return st.decode("utf-8")

proc = subprocess.Popen(["./../../NAR","shell"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
irc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
irc.connect((server, 6667))
irc.send(encode("USER "+ botnick +" " + botnick +" "+ botnick +" :Hallo!\r\n"))
irc.send(encode("NICK "+ botnick +"\r\n"))
irc.send(encode("PRIVMSG nickserv :iNOOPE\r\n"))
irc.send(encode("JOIN "+ channel +"\r\n")) #fails on euirc, works in all others, euirc is not completely RFC 2810 compatible maybe?
print("connected")
print("NARS instance created")
print("Narsese Filter applied: '"+str(Narsese_Filter)+"'")

def receive_thread(a):
    while True:
       msg=proc.stdout.readline()
       if msg!=None and msg!="" and True in [u in msg for u in Narsese_Filter]:
          print("NAR output: "+msg)
          irc.send(encode("PRIVMSG "+ channel +" : "+msg+"\r\n"))
thread.start_new_thread(receive_thread,(1,))

while True:
    try:
        text=decode(irc.recv(2040))
        print(text)
        if "PING" in text:
            STR='PONG :' + text.split("PING :")[1].split("\n")[0] + '\r\n';
            irc.send(encode(STR))
        else:
            if "VERSION" in text or "End of message of the day" in text:
                irc.send(encode("JOIN "+ channel +"\r\n")) #join when version private message comes :D
            else:
                TEXT=text.replace(":|:","__REPLACE_EVENT_MARKER__").split(":")[-1].replace("__REPLACE_EVENT_MARKER__", ":|:")
                if TEXT.replace(" ","").replace("\n","").replace("\r","")=="":
                    continue
                if TEXT.startswith("*reset"):
                    proc.stdin.write("*reset\n")
                if TEXT.startswith("nars: ") or TEXT.startswith("(") or TEXT.startswith("<") or TEXT.startswith("100") or " :|:" in TEXT:
                    if TEXT.startswith("nars: "):
                        TEXT = TEXT.split("nars: ")[1]
                    print("NAR input: "+TEXT)
                    try:
                        proc.stdin.write(TEXT+"\n")
                        proc.stdin.flush()
                    except:
                        None
    except Exception as exception:
        print("exception" + str(exception))
None
