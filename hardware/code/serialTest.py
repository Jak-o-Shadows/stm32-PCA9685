import serial
import time

port = "COM5"
baud = 9600

ser = serial.Serial()
ser.port = port
ser.baudrate = baud

data1 = [[0xFF, i, 0x00] for i in xrange(2)]
data2 = [[0xFF, i, 0xFF] for i in xrange(2)]
data3 = [[0xFF, i+32, 0x7F] for i in xrange(2)]


d1 = [item for sublist in data1 for item in sublist]
d2 = [item for sublist in data2 for item in sublist]
d3 = [item for sublist in data3 for item in sublist] + [0xFF, 0xFF, 0xFF]

ser.open()
for c in xrange(10):
    print d1
    for i in d1:
        ser.write(chr(i))

    time.sleep(1)
    print d2
    for i in d2:
        ser.write(chr(i))
    time.sleep(1)
    print d3
    for i in d3:
        ser.write(chr(i))
    time.sleep(1)

