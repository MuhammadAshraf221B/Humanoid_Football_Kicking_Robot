import socket
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.sendto(b"150\n", ("10.229.235.220", 4210))