import binascii

import serial

from typing import Tuple, List, Sequence

class ConnectionWrapper():
    def __init__(self, con):
        self.con = con

    def send(self, num:int):
        uint8_t = hex(num)[2:].zfill(2)
        b1 = binascii.unhexlify(uint8_t)
        self.con.write(b1)

    def sendBinary(self, b: bytes):
        self.con.write(b)

    def getBinary(self) -> bytes:
        return self.con.read(1)


class Uint8_t():
    def __init__(self, numeric: int):
        self.numeric: int = numeric

    @property
    def hexDigits(self) -> str:
        return hex(self.numeric)[2:].zfill(2)

    @property
    def bin(self) -> bytes:
        return binascii.unhexlify(self.hexDigits)
        
    @property
    def raw(self) -> int:
        return self.numeric

class RueP():

    # Define Extra
    ALL: Uint8_t = Uint8_t(256)
    
    # Define Commands
    LISTEN: Uint8_t =         Uint8_t(0)
    IGNORE: Uint8_t =         Uint8_t(1)
    LISTENONCE: Uint8_t =     Uint8_t(2)
    SETFLAGS: Uint8_t =       Uint8_t(3)
    # Define flags for SETFLAGS
    EXECPOS: Uint8_t =        Uint8_t(0)
    POWERUP: Uint8_t =        Uint8_t(1)
    POWERDOWN: Uint8_t =      Uint8_t(2)
    PRESET: Uint8_t =         Uint8_t(3)
    # Define more commands
    SETPOSITION: Uint8_t =    Uint8_t(4)
    CACHEPOS: Uint8_t =       Uint8_t(5)
    GETCURRENT: Uint8_t =     Uint8_t(6)
    GETPOSITION: Uint8_t =    Uint8_t(7)
    GETMODEL: Uint8_t =       Uint8_t(8)

    def __init__(self):
        pass
        

    def connect(self, port: str, baudrate: int):
        con = serial.Serial()
        con.port = port
        con.baudrate = baudrate
        con.open()
        self.con = ConnectionWrapper(con)

    def prepareCommand(self, command: Uint8_t, argument: Uint8_t) -> Tuple[Uint8_t, Uint8_t]:
       byte1 = Uint8_t( (command.raw << 3) | (argument.raw & 0x07) )
       byte2 = Uint8_t( (argument.raw >> 3 ) | 0x80 )
       
       return [byte1, byte2]

    def sendCommand(self, command: Uint8_t, argument: Uint8_t):
        byte1, byte2 = self.prepareCommand(command, argument)
        self.con.sendBinary(byte1.bin)
        self.con.sendBinary(byte2.bin)

    def exchange(self, data: Sequence[Uint8_t], count: int, wait_us: int) -> List[bytes]:
        """
        Exchange data - WIP
        """
        # Write out
        for num in data:
            self.con.sendBinary(num.bin)

        # Read back
        returnData: bytes = []
        for byteNum in range(count):
            # TODO: timeout on wait_us*count
            returnData.append(self.con.getBinary())
            
        return returnData


    # Higher Level Commands

    def dismissAll(self):
        """
        Dismiss all servos from listening
        """
        self.sendCommand(self.IGNORE, self.ALL)

    def positionCacheExecute(self):
        """
        Execute the cached position for all
        """
        # Move to cached position command
        #   Note that we don't need to issue a 'listen' command here
        self.sendCommand(self.SETFLAGS, Uint8_t(0x01 << self.EXECPOS.numeric))

    def positionCache(servoIdx, newPos):
        self.sendCommand(self.LISTENONCE, protocol.Uint8_t(servoIdx))
        self.sendCommand(self.CACHEPOS,   protocol.Uint8_t(newPos))































