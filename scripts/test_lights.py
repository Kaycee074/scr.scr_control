import socket

lights = "192.168.0.111"
port = 57007

cmdstr = "PS0000FFFF000000000000"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((lights,port))
s.send(cmdstr)
s.close()
