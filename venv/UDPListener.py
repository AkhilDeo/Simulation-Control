import socket
import json
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

    #if len(dataArray) == 2:
     #   gripper = dataArray[1]
        #Set grip/pincer according to this value
    #else:
        # Need to case strings to doubles / bool
     #   x = dataArray[1]
      #  y = dataArray[3]
       # z = dataArray[5]
        #roll = dataArray[7]
        #pitch = dataArray[9]
        #yaw = dataArray[11]
        #gripper = dataArray[13]
        #cameraMoving = dataArray[15]
    # Set AMBF simulation values based on these values

    # Maybe once cameraMoving = true, set 1st pos as "origin" and
    # move camera based on that first home/origin val


