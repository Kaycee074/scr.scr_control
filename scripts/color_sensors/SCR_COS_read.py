import socket, sys, time

def establish_connection(address, port):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((address, port))
	return s

if __name__ == "__main__":
	data = ''
	while (len(data) != 2053):
		s = establish_connection(str(sys.argv[1]), int(sys.argv[2]))
		s.send("CS_Rall".encode())
		data = s.recv(2053).decode()
		s.shutdown(socket.SHUT_RDWR)
		s.close()

	print(data)