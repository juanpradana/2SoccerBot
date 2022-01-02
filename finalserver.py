import socket, random
import os
from _thread import *

ServerSocket = socket.socket()
host = '10.10.12.29'
port = 7645
ServerSocket.settimeout(0.1)
ThreadCount = 0
kuota = 0
nah = ''
nahlo = ''
try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))

print('Waitiing for a Connection..')
ServerSocket.listen(2)

def threaded_client1(connection):
    global nah
    while True:
        try:
            while(1):
                z1 = (connection.recv(32)).decode('utf-8')
                if not z1:
                    break
                elif z1:
                    nah = z1
        except:
            pass
        connection.sendall(bytes(nahlo, encoding='utf8'))
    connection.close()

def threaded_client2(connection):
    global nahlo
    while True:
        try:
            while(1):
                z2 = (connection.recv(32)).decode('utf-8')
                if not z2:
                    break
                elif z2:
                    nahlo = z2
        except:
            pass
        connection.sendall(bytes(nah, encoding='utf8'))
    connection.close()

def perintah():
    global nah, nahlo
    #nah  = readchar.readkey()
    while True:
        masuk = str(input("masukkan perintah:\n"))
        if masuk == 'start':
            rand = random.randint(1, 2)
            if rand == 1:
                nah = 'bot1start'
                nahlo = 'bot1start'
            else:
                nahlo = 'bot2start'
                nah = 'bot2start'
        else:
            pass
        #c.sendall(input.encode())
start_new_thread(perintah, ())
while True:
    
    if kuota == 0:
        while(1):
            try:
                Client, address = ServerSocket.accept()
                break
            except:
                continue
        #print(Client)
        print('Connected to: ' + address[0] + ':' + str(address[1]))
        start_new_thread(threaded_client1, (Client, ))
        kuota += 1
        ThreadCount += 1
    elif kuota == 1:
        while(1):
            try:
                Client, address = ServerSocket.accept()
                break
            except:
                continue
        #print(Client)
        print('Connected to: ' + address[0] + ':' + str(address[1]))
        start_new_thread(threaded_client2, (Client, ))
        kuota += 1
        ThreadCount += 1       
        
    #print('Thread Number: ' + str(ThreadCount))
ServerSocket.close()

"""tambahin stop"""