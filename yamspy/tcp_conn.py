import socket

class TCPSocket:
    """TCP Socket for Software In The Loop (SITL)
    """

    def close(self):
        if not self.sock:
            raise Exception("Cannot close, socket never created")
        self.closed = True
        self.sock.close()

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            self.buffersize = self.sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
        else:
            self.sock = sock
        self.closed = False
        self.timeout_exception = socket.timeout

    def connect(self, host='127.0.0.1', port=54320, timeout=1/500):
        self.sock.connect((host, port))
        self.sock.settimeout(timeout)
        self.closed = False

    def send(self, msg):
        sent = self.sock.send(msg)
        if not sent:
            raise RuntimeError("socket connection broken")
        return sent

    def receive(self, size = None):
        recvbuffer = b''
        try:
            if size:
                recvbuffer = self.sock.recv(size)
            else:
                recvbuffer = self.sock.recv(self.buffersize)
        except socket.timeout:
            return recvbuffer
        if not recvbuffer:
            raise RuntimeError("socket connection broken")

        return recvbuffer
