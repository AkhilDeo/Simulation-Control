import socket
import json
from ambf_client import Client
import time

_client = Client()
_client.connect()
print(_client.get_obj_names())

UDP_IP = socket.gethostbyname(socket.gethostname())
UDP_PORT = 15002
#print("Start")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
while True:
    data, addr = sock.recvfrom(1024)
    #data = data.decode()
    if bool(data):
        dataDict = json.loads(data)
        print(dataDict)
        print(dataDict['slider'])