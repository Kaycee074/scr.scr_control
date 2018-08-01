import socket, sys

def establish_connection(address, port):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((address, port))
	return s

if __name__ == "__main__":
	s = establish_connection(sys.argv[1], sys.argv[2])
	s.send("CS_Rall".encode())
	data = s.recv(2053).decode()
	s.close()
	print(data)