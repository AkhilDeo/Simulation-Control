import socket
UDP_IP = socket.gethostbyname(socket.gethostname()) # or UDP_IP = socket.gethostbyname(socket.gethostname())
UDP_PORT = 15002
print("Start")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
while True:
    data, addr = sock.recvfrom(1024)
    data = data.decode()
    dataArray = data.split(" ")


    print(data)


