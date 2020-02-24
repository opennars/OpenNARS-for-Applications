import socket

#./NAR UDPNAR 127.0.0.1 50000 100000 true

line = "//justcomment"

ip = "127.0.0.1"
port = 50000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(line.encode(), (ip, port))
