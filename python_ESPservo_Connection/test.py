import socket
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.sendto(b"132\n", ("192.168.1.11", 4210))