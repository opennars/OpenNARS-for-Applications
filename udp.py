import socket

line = "<a-->b>."

ip = "127.0.0.1"
port = 50000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(line.encode(), (ip, port))
