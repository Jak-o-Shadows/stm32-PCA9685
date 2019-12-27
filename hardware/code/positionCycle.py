
import math
import itertools
from collections import namedtuple

import time

import serialTest as protocol

def angleToValue(angle_deg: float) -> int:
    minValue = 100+30
    maxValue = 400+30
    midValue = 306+30  # 1.5ms

    
    minAngle_deg = 0
    midAngle_deg = 90
    maxAngle_deg = 180
    
    # Hence convert
    valuePerDegree = (maxValue-minValue)/(maxAngle_deg-minAngle_deg)
    value = (angle_deg-minAngle_deg)*valuePerDegree + minValue
    
    return int(math.floor(value))
    

def dismissAll():
    """
    Dismiss all servos from listening
    """
    protocol.sendCommand(protocol.IGNORE, protocol.Uint8_t(256))

def positionCacheSet(posCollection):
    """
    Set the position cache for the various positions
    """
    # Dismiss all first, as some may be previously listening
    dismissAll()
    
    # Then set cache for each
    for pos in posCollection:
        print(pos)
        value = angleToValue(pos.angle_deg)
        print(pos.angle_deg, value)
        
        # Do listen once command for each servo
        for servoID in pos.servoIDs:
            # Set listen once for this servo
            protocol.sendCommand(protocol.LISTENONCE, protocol.Uint8_t(servoID))
        
        # Do set position cache for each servo
        protocol.sendCommand(protocol.CACHEPOS, protocol.Uint8_t(value))
    
    # Be a good neighbour and dismiss all
    dismissAll()

def positionCacheExecute(servoIDs):
    """
    Execute the cached position for the specified servoIDs
    """
    # Move to cached position command
    #   Note that we don't need to issue a 'listen' command here
    protocol.sendCommand(protocol.SETFLAGS, protocol.Uint8_t(0x01 << protocol.EXECPOS.numeric))

if __name__ == "__main__":
    
    Position = namedtuple("Position", ["angle_deg", "servoIDs"])
    
    pos1 = [Position(90, [0, 1, 2, 3, 4, 5, 6, 7])]
    pos2 = [Position(90+30, [0, 2, 4, 6]),
            Position(90-30, [1, 3, 5, 7])]
    positions = [pos1, pos2]

    positions = []
    for x in [45, 90, 90+45]:#range(45, 90+45, 5):
        posEven = Position(    x, list(range(0, 16, 2)))
        posOdd =  Position(180-x, list(range(1, 16, 2)))
        positions.append([posEven, posOdd])
    for x in [45, 90, 90+45]:#range(45, 90+45, 5):
        posEven = Position(180-x, list(range(0, 16, 2)))
        posOdd =  Position(    x, list(range(1, 16, 2)))
        positions.append([posEven, posOdd])

    for repeatIdx in range(10):
        for pos in positions:
            print("")
            print(pos)
            #input("\tSet?")
            positionCacheSet(pos)
            #input("\tExecute?")
            positionCacheExecute(itertools.chain(*[x.servoIDs for x in pos]))
            time.sleep(0.5)


    