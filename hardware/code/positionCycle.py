
import math
import itertools
from collections import namedtuple

import time

import serialTest as protocol

def angleToValue(angle_deg: float) -> int:
    minValue = 204+30
    midValue = 306+30
    maxValue = 408+30
    
    minAngle_deg = 0
    minAngle_deg = 90
    maxAngle_deg = 180
    
    # Hence convert
    valuePerDegree = (maxValue-minValue)/(maxAngle_deg-minAngle_deg)
    value = (angle_deg-minAngle_deg)*valuePerDegree + minValue
    
    return int(math.floor(value))
    
    

def positionCacheSet(posCollection):
    """
    Set the position cache for the various positions
    """
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

def positionCacheExecute(servoIDs):
    """
    Execute the cached position for the specified servoIDs
    """
    for servoID in servoIDs:
        # Set listen once for this servo
        protocol.sendCommand(protocol.LISTENONCE, protocol.Uint8_t(servoID))

    # Move to cached position command
    protocol.sendCommand(protocol.SETFLAGS, protocol.Uint8_t(0x01 << protocol.EXECPOS.numeric))

if __name__ == "__main__":
    
    Position = namedtuple("Position", ["angle_deg", "servoIDs"])
    
    pos1 = [Position(90, [1, 2, 3, 4, 5, 6, 8, 9])]
    pos2 = [Position(90+30, [1, 3, 5, 9]),
            Position(90-30, [2, 4, 6, 8])]


    for repeatIdx in range(5):
        for pos in [pos1, pos2]:
            print("")
            positionCacheSet(pos)
            #input("\tExecute?")
            positionCacheExecute(itertools.chain(*[x.servoIDs for x in pos]))
            time.sleep(5)


    