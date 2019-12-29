import serial
import time
import binascii

import protocol

from typing import Tuple, List, Sequence




port = "COM8"
baud = 9600

p = protocol.RueP()
p.connect(port, baud)


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
    p.sendCommand(p.LISTENONCE, protocol.Uint8_t(argument))

def menu2():
    """
    Call servo to attention
    """
    argument = -1
    while (argument < 0) or (argument > 256):
        argument = int(input("Which servo would you like to call on? (256 for all)  "))
    p.sendCommand(p.LISTEN, protocol.Uint8_t(argument))

def menu3():
    """
    Dismiss Servo
    """
    argument = -1
    while (argument < 0) or (argument > 256):
        argument = int(input("Which servo would you like to call off? (256 for all)  "))
    p.sendCommand(p.IGNORE, protocol.Uint8_t(argument))
    
def menu6():
    """
    Set Servo Position
    """
    argument = -1
    while (argument < 0) or (argument > 1025):
        argument = int(input("What position? (0 to 1023) "))
    p.sendCommand(p.SETPOSITION, protocol.Uint8_t(argument))

def menu9():
    """
    Execute cache position for all servos
    """
    p.sendCommand(p.SETFLAGS, protocol.Uint8_t(0x01 << p.EXECPOS.numeric))

def menu10():
    """
    Set Cached Position
    """
    argument = -1
    while (argument < 0) or (argument > 1025):
        argument = int(input("What position? (0 to 1023) "))
    p.sendCommand(p.CACHEPOS, protocol.Uint8_t(argument))
    
if __name__ == "__main__":
    mainMenu()




