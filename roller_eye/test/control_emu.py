import socket
import sys
import tty
import termios

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == "__main__":
    sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    remote="localhost"
    if len(sys.argv)>=2:
        remote=sys.argv[1]
    sock.connect((remote,60000))
    while True:
        c=readchar()
        if c=='q':
            sock.send(b'v -1.0  0\n')
        elif c=="e":
            sock.send(b'v 1.0 0 \n')
        elif c== "w":
            sock.send(b"v 0 1.0\n")
        elif c=="s":
            sock.send(b"v 0 -1.0\n")
        elif c=="a":
            sock.send(b"w 1.0\n")
        elif c=="d":
            sock.send(b"w -1.0\n")
        elif c=="\033":
            break
    
    sock.close()
