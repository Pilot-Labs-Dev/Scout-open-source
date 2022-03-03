#!/usr/bin/python3

import RPi.GPIO as gpio
import traceback
import sys
import tty
import termios
import serial
import _thread
import time

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

gpio.setmode(gpio.BOARD)

class Motor:
    def __init__(self,en,int1,int2,desc):
        self.en=en
        self.int1=int1
        self.int2=int2
        self.desc=desc

        gpio.setup(en,gpio.OUT,initial=gpio.LOW)
        self.pwm=gpio.PWM(en,200)
        gpio.setup(int1,gpio.OUT,initial=gpio.HIGH)
        gpio.setup(int2,gpio.OUT,initial=gpio.LOW)

        self.started=False
        self.duty=0
        self.dir=True
        self.lastDir=False
        self.lastDuty=0
    
    def setParam(self,speed,dir):
        if speed<0:
            speed=0.0
        
        self.duty=int(49.6+speed/10*50)
        if self.duty>100:
            self.duty=100
        
        self.dir=dir

    def flush(self):
        if self.duty==self.lastDuty and self.dir==self.lastDir and self.started:
            return
        
        print("%s: %s %d"%(self.desc,"P" if self.dir else "N",self.duty))

        if self.duty<50:
            gpio.output(self.int1,gpio.LOW)
            gpio.output(self.int2,gpio.LOW)
        else:
            gpio.output(self.int1,gpio.HIGH if self.dir else gpio.LOW)
            gpio.output(self.int2,gpio.LOW if self.dir else gpio.HIGH)  

        if self.started:
            self.pwm.ChangeDutyCycle(self.duty)
        else:
            self.pwm.start(self.duty)
            self.started=True

        self.lastDuty=self.duty
        self.lastDir=self.dir

    def powerOn(self):
        print("power on")
    
    def powerOff(self):
        print("power off")
        self.stop()
        self.duty=0
        self.dir=True

    def start(self):
        if not self.started:
            self.flush()
        else:
            print("alread started")

    def stop(self):
        if self.started:
            self.pwm.stop()
        else:
            print("already stop")
        self.started=False

        
        
engine=(Motor(12,16,18,"Motor0"),Motor(32,38,36,"Motor1"),Motor(33,15,13,"Motor2"),Motor(35,29,31,"Motor3"))

def doCmd(cmd):

    badStr="Bad Cmd"
    outRange="Motor outof range"
    unkonwCmd="unkonw cmd"

    items=cmd.split()

    if len(items)<1:
        return False,badStr

    if items[0]=="F":
        for m in engine:
            m.flush()
    elif items[0]=="PO":
        for m in engine:
            m.powerOn()
    elif items[0]=="PF":
        for m in engine:
           m.powerOff()
    elif items[0]=="P":
        for m in engine:
           m.stop()
    elif items[0]=="R":
        for m in engine:
            m.start()
    elif items[0]=="S":
        try:
            idx=int(items[1])
            if idx<0 or idx>=4:
                return False,outRange

            engine[idx].setParam(float(items[3]),bool(int(items[2])))
        except:
            return False,badStr
    else:
        return False,unkonwCmd
    
    return True,""

def setParam(pars):
    for idx in range(0,4):
        engine[idx].setParam(pars[idx][1],pars[idx][0])

    print("----------------")
    for idx in range(0,4):  
       engine[idx].flush()

def up(speed):
    setParam([(True,speed),(True,speed),(True,speed),(True,speed)])

def down(speed):
    setParam([(False,speed),(False,speed),(False,speed),(False,speed)])

def right(speed):
    setParam([(False,speed),(True,speed),(True,speed),(False,speed)])

def left(speed):
    setParam([(True,speed),(False,speed),(False,speed),(True,speed)])

def roll_anti_clock(speed):
    setParam([(True,speed),(False,speed),(True,speed),(False,speed)])

def roll_clock(speed):
    setParam([(False,speed),(True,speed),(False,speed),(True,speed)])


def test():
    speed=1.0
    count=0
    try:
        while True:
            c=readchar()
            if c=='q':
                left(speed)
            elif c=="e":
               right(speed)
            elif c== "w":
               up(speed)
            elif c=="s":
                down(speed)
            elif c=="a":
                roll_anti_clock(speed)
            elif c=="d":
                roll_clock(speed)
            elif c=="\033":
                break
            else: 
                break
            count=count+1
            if count%5==0:
                speed=speed+1.0
    except Exception as e:
        print(e)
        traceback.print_exc()

def test2():
    while True:
        cmd=input("")
        print("recieve cmd:",cmd)
        ret,err=doCmd(cmd) 
        if not ret:
            if cmd=="Q":
                break
            print(err) 

def heardbeat(s):
    while True:
        s.write("motor hello\r\n".encode("utf8"))
        time.sleep(5)

def serialServe():
    try:
        s=serial.Serial("/dev/serial0",115200,timeout=10)
        _thread.start_new_thread(heardbeat,(s,))
        while True:
            cmd=s.readline()
            try:
                cmd=str(cmd,encoding="utf8").replace("\r","").replace("\n","")
            except:
                continue
            
            print("recieve :",cmd)
            doCmd(cmd)
    except Exception as e:
        print(e)
        print("service exit!!!")


if __name__ == "__main__":
    #test1()
    #test2()
    serialServe()
    gpio.cleanup()
