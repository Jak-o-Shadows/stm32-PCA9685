import serial
import time
import binascii

from typing import Tuple, List, Sequence


def send(con, num):
    uint8_t = hex(num)[2:].zfill(2)
    #input("continue?")
    b1 = binascii.unhexlify(uint8_t)
    print(uint8_t, b1)
    con.write(b1)
    
def sendBinary(con, b: bytes):
    con.write(b)
    
def getBinary(con) -> bytes:
    return con.read(1)

port = "COM8"
baud = 9600

con = serial.Serial()
con.port = port
con.baudrate = baud
con.open()


class Uint8_t():
    def __init__(self, numeric: int):
        self.numeric = numeric

    @property
    def hexDigits(self) -> str:
        return hex(self.numeric)[2:].zfill(2)

    @property
    def bin(self) -> bytes:
        return binascii.unhexlify(self.hexDigits)
        
    @property
    def raw(self) -> int:
        return self.numeric

LISTEN =         Uint8_t(0)
IGNORE =         Uint8_t(1)
LISTENONCE =     Uint8_t(2)
SETFLAGS =       Uint8_t(3)

EXECPOS =        Uint8_t(0)
POWERUP =        Uint8_t(1)
POWERDOWN =      Uint8_t(2)
PRESET =         Uint8_t(3)

SETPOSITION =    Uint8_t(4)
CACHEPOS =       Uint8_t(5)
GETCURRENT =     Uint8_t(6)
GETPOSITION =    Uint8_t(7)
GETMODEL =       Uint8_t(8)


        
def prepareCommand(command: Uint8_t, argument: Uint8_t) -> Tuple[Uint8_t, Uint8_t]:
   byte1 = Uint8_t( (command.raw << 3) | (argument.raw & 0x07) )
   byte2 = Uint8_t( (argument.raw >> 3 ) | 0x80 )
   
   return [byte1, byte2]

def sendCommand(command: Uint8_t, argument: Uint8_t):

    byte1, byte2 = prepareCommand(command, argument)
    sendBinary(con, byte1.bin)
    sendBinary(con, byte2.bin)


def exchange(data: Sequence[Uint8_t], count: int, uswait: int) -> List[bytes]:

    # Write out
    for num in data:
        sendBinary(con, num.bin)

    # Read back
    returnData: bytes = []
    for byteNum in range(count):
        # TODO: timeout on uswait*count
        returnData.append(getBinary(con))
        
    return returnData



############# Menu System##########
def textcolor(arg):
    pass

def mainMenu():

    functionDict = {1:  menu1,
                    2:  menu2,
                    3:  menu3,
                    6:  menu6,
                    9:  menu9,
                    10: menu10}

    choice = -1
    while( choice != 99):
        choice = -1
        while( choice < 1  or choice > 99):
            print(choice)
            print("\n\
          1) One time listen and chain address\n\
          2) Call servo to attention\n\
          3) Dismiss servo\n\
          4) Power up\n\
          5) Power down\n\
          6) Set position\n\
          7) Get position\n\
          8) Get device model\n\
          9) Goto cached position\n\
         10) Set cached position\n\
         11) Preset to current position\n\
         12) Get current\n\
         13) Autodetect servos \n\
         14) Position list \n\
         15) Current list \n\
         99) Exit.\n\
         \n")
            textcolor(1)
            print("  Selection > ")
            textcolor(7)
            choice = input(":")
            if choice == "":
                choice = 99
            else:
                choice = int(choice)
            textcolor(2)
    
        try:
            fcn = functionDict[choice]
        except KeyError as e:
            print("Choice not valid")
            raise e
        fcn()

def menu1():
    """
    Set one time listen
    """
    argument = -1
    while (argument < 0) or (argument > 768):
        argument = int(input("Which servo would you like to call on? (256 for all) (add 512 for chain select) "))
    sendCommand(LISTENONCE, Uint8_t(argument))

def menu2():
    """
    Call servo to attention
    """
    argument = -1
    while (argument < 0) or (argument > 256):
        argument = int(input("Which servo would you like to call on? (256 for all)  "))
    sendCommand(LISTEN, Uint8_t(argument))

def menu3():
    """
    Dismiss Servo
    """
    argument = -1
    while (argument < 0) or (argument > 256):
        argument = int(input("Which servo would you like to call off? (256 for all)  "))
    sendCommand(IGNORE, Uint8_t(argument))
    
def menu6():
    """
    Set Servo Position
    """
    argument = -1
    while (argument < 0) or (argument > 1025):
        argument = int(input("What position? (0 to 1023) "))
    sendCommand(SETPOSITION, Uint8_t(argument))

def menu9():
    """
    Execute cache position for all servos
    """
    sendCommand(SETFLAGS, Uint8_t(0x01 << EXECPOS.numeric))

def menu10():
    """
    Set Cached Position
    """
    argument = -1
    while (argument < 0) or (argument > 1025):
        argument = int(input("What position? (0 to 1023) "))
    sendCommand(CACHEPOS, Uint8_t(argument))
    
if __name__ == "__main__":
    mainMenu()




