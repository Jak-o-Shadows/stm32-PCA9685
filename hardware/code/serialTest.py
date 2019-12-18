import serial
import time
import binascii


def send(con, num):
    uint8_t = hex(num)[2:].zfill(2)
    #input("continue?")
    b1 = binascii.unhexlify(uint8_t)
    print("\t", b1)
    con.write(b1)

port = "COM8"
baud = 9600

ser = serial.Serial()
ser.port = port
ser.baudrate = baud

data1 = [[0xFF, i, 0x00] for i in range(2)]
data2 = [[0xFF, i, 0xFF] for i in range(2)]
data3 = [[0xFF, i+32, 0x7F] for i in range(2)]


d1 = [item for sublist in data1 for item in sublist]
d2 = [item for sublist in data2 for item in sublist]
d3 = [item for sublist in data3 for item in sublist] + [0xFF, 0xFF, 0xFF]

ser.open()
for c in range(4):
    print(d1)
    for i in d1:
        send(ser, i)

    time.sleep(3)
    print(d2)
    for i in d2:
        send(ser, i)
    time.sleep(3)
    print(d3)
    for i in d3:
        send(ser, i)
    time.sleep(10)

