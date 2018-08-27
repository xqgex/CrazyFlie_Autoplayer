import sys
import socket
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 8080        # The port used by the server

if len(sys.argv) < 3:
    print("[USAGE1]   robot.txt   sites.txt   obstacles.txt")
    exit()

cmds = sys.argv[1:]

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print("connected ")

    with open(cmds[0], 'r') as myfile:
        data = myfile.read()
    #print(data)
    s.sendall(data.encode())
    respond = s.recv(1024)
    print("sent robot")
    #if(respond != 1):
    #    exit()

    with open(cmds[1], 'r') as myfile:
        data = myfile.read()
    #print(data)
    s.sendall(data.encode())
    respond = s.recv(1024)
    print("sent sites")

    with open(cmds[2], 'r') as myfile:
        data = myfile.read()
    #print(data)
    s.sendall(data.encode())
    respond = s.recv(1024)
    print("sent obstacles")

    s.sendall(b'hello')

    respond = s.recv(1024)
    print("path returned is: " + respond.decode())

