# -*- coding: utf-8 -*-
"""
Created on Tue Jul 09 17:15:30 2013
@author: Jak
Handles the connection and queues and threading
"""
import socket
import threading
import time
#import io
import sys
import binascii

class Protocol():
    """Implements the actual send/recieve interface"""
    def __init__(self):
        pass
    

def message(data):
    """Converts some data into the stuff that gets sent over the connection
    Can replace me! Use if changing line ending for example.
    Copy the bytes(str(data)+ etc), etc though
    """
    print("Message is:",data)
    try:
        return bytes(str(data) + ";", "utf-8")
    except TypeError:
        return bytes(str(data) + ";")
    
    
try:
    if sys.version_info.major >2:
        import queue   
        class BluetoothProtocol():
            """Connects to a bluetooth to serial adaptor"""
            def __init__(self):
                self.MAC = "00:00:00:00:00:00"
                self.port = 3
            
            def connect(self):
                #self.con = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.con = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.con.connect((self.MAC, self.port))
                
            def disconnect(self):        
                raise NotImplementedError("Disconnect not implemented for bluetooth socket" )
                
            def send(self, msg):
                self.con.send(msg)
                
            def read(self):
                pass
    else:
        import Queue as queue
        import bluetooth
        class BluetoothProtocol():
            """Connects to a bluetooth to serial adaptor using pybluez"""
            def __init__(self):
                self.MAC = "00:00:00:00:00:00"
                self.port = 3
            
            def connect(self):
                #self.con = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.con = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
                self.con.connect((self.MAC, self.port))
                
            def disconnect(self):
                raise NotImplementedError("Disconnect not implemented for pyBluez bluetooth socket" )
                
            def send(self, msg):
                self.con.send(msg)
                
            def read(self):
                pass
except ImportError:
    pass
            

try:
    import serial
    class SerialProtocol(Protocol):
        """Serial protocol, defaults to COM3 and 9600br"""
        def __init__(self):
            self.port = "COM3"
            self.baudrate = 9600
            
            self.eol = "\n"
            
            self.buffer = []
            self.connected = False
            
            self.logFile = None
            
            self.type = "SERIAL"
            self.dSize = 1
        
        def connect(self):
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate)
            self.connected=True
            #self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, newline=self.eol))
            
        def openLog(self, fname):
            if not self.logFile:
                self.logFile = open(fname, "wb")
            else:
                raise Exception("Logfile already open")
            
        def disconnect(self):
            self.ser.close()
            self.connected=False
            if self.logFile:
                self.logFile.close()
            
            
        
        def send(self, msg):
            #self.sio.write(unicode(msg))
            #self.ser.write(unicode(msg))
            binary = binascii.unhexlify(msg)
            self.ser.write(binary)
            if self.logFile:
                self.logFile.write("O:" + hex(ord(msg)) + "\n")
                self.logFile.flush()

        
        def readBit(self):
            data = self.readByte
            
            if data != self.eol:
                print(ord(data))
                print(hex(ord(data)))
                print(str(hex(ord(data))))
                self.buffer.append(str(hex(ord(data)))[2:])
                return True
            else:
                return False
                
        def readByte(self):
            data = self.ser.read(1)
            if self.logFile:
                self.logFile.write("I:" + str(hex(ord(data)))[2:].zfill(2) + "\n")
                self.logFile.flush()
            return data
                
        def read(self):
            
            keepGoing = True
            while keepGoing:
                status = self.readBit()
                if not status:
                    data = self.buffer
                    self.buffer = []
                    return "".join(data)
            
            
except ImportError:
    print("Serial Unavailable")

   
    
try:
    import usb
    class USBProtocol(Protocol):
        """
        USB protocol, defaults to 64 byte, VID 0x0483, PID 0x5710
        send endpoint of 0x01, recv endpoint of 0x81
        """
        def __init__(self):
            self.vid = 0x0483
            self.pid = 0x5710
            self.dSize = 64
            self.rxEP = 0x81
            self.txEP = 0x01
            self.timeout = 1500#ms
                        
            self.connected = False
            
            self.logFile = None
            
            self.type = "USB"
        
        def connect(self):
            self.dev = usb.core.find(idVendor = self.vid, idProduct=self.pid)
            self.dev.set_configuration()
            #read one because the first lot is junk
            self.readByte()
            self.connected=True
            
        def openLog(self, fname):
            if not self.logFile:
                self.logFile = open(fname, "wb")
            else:
                raise Exception("Logfile already open")
            
        def disconnect(self):
            self.dev.close()
            self.connected=False
            if self.logFile:
                self.logFile.close()
            
            
        
        def send(self, msg):
            data = self.dSize*[0]
            data[0] = ord(msg)
            #print(data)
            self.dev.write(self.txEP, data, self.timeout)
            if self.logFile:
                self.logFile.write("O:" + hex(ord(msg)) + "\n")
                self.logFile.flush()


                
        def readByte(self):
            """
                USB is a packet protocol - actually reads a packet
                Not renamed cause I'm lazy
            """
            data = self.dev.read(self.rxEP, self.dSize, self.timeout)
            #print("rx", data)
            data = [chr(x) for x in data]
            if self.logFile:
                for i in data:
                    self.logFile.write("I:" + str(hex(ord(i)))[2:].zfill(2) + "\n")
                    self.logFile.flush()
            return data
    
    
    
except ImportError as e:
    print("USB Unavailable")
    print(e)
    

class SocketProtocol(Protocol):
    """Send and recieve messages over an tcp/ip socket
    Change self.address = (adddr, port). Defaults to ('localhost', 8080)
    """
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.address = ("localhost", 8080)
              
        
    def connect(self):
        self.sock.connect(self.address)
        
    def disconnect(self):
        self.sock.close()
        
    def send(self, msg):
        self.sock.send(msg)
        
    def read(self):
        l = self.sock.recv(4096)
        return l

class SocketServer(Protocol):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.address = ("localhost", 8080)
        
    def connect(self):
        """
        Serve as a socket server
        """
        self.sock.bind(self.address)
        self.sock.listen(1)
        
        self.con, self.addr = self.sock.accept()
        
    def send(self, msg):
        self.con.sendall(msg)
        
    def read(self):
        pass
        #print("read not implemented")
        
        
class ThreadHelper():
    """Manages the thread.
    Use ThreadHelper.q.put(item) to put stuff onto the queue for the thread
    """
    def __init__(self, protocol, message):
        self.sendQueue = queue.Queue()
        self.recvQueue = queue.Queue()
        self.protocol = protocol()
        self.msg = message
        
    def startThread(self):
        t = threading.Thread(target=self.sendThread)
        t.daemon = True
        t.start()
        r = threading.Thread(target=self.recvThread)
        r.daemon = True
        r.start()
    
    def sendThread(self):
        keepGoing = True
        while keepGoing:
            item = self.sendQueue.get()
            self.protocol.send(self.msg(item))
            time.sleep(0.01)
            self.sendQueue.task_done()
            
    def recvThread(self):
        keepGoing = True
        keepGoing = False
        while keepGoing:
            data = self.protocol.read()
            try:
                #python 3
                data = str(data, 'utf-8')
            except:
                #python 2.7x
                data = str(data)
            self.recvQueue.put(data)