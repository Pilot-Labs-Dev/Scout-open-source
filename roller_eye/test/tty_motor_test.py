#!/usr/bin/python3

import serial
import sys

testMove=False

if len(sys.argv) >=2:
    serial_path=sys.argv[1]
else:
    serial_path="/dev/ttyUSB0"

if len(sys.argv)>=3:
    testMove=True

s=serial.Serial(serial_path,115200)
cout=0
while True:
    r=s.readline()
    print("recieve:",str(s.readline(),encoding="utf8").replace("\r","").replace("\n",""))

    if not testMove:
        continue

    if cout%3==0:
        s.write(("S 0 1 %s\n"%(cout)).encode("utf8"))
    elif cout%3==1:
        s.write("F\n".encode("utf8"))
    else:
        s.write("PF\n".encode("utf8"))

    cout=cout+1
