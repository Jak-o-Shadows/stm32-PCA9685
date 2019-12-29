
import math
from collections import namedtuple
import time

import protocol

port = "COM8"
baud = 9600

p = protocol.RueP()
p.connect(port, baud)



def angleToValue(angle_deg: float) -> int:
    minValue = 30 + int(0.9/5e-3)  # Offset + counts - 5us per count, 0.9ms pulse
    maxValue = 30 + int(2.14/5e-3)  # Offset + counts - 5us per count, 2.14ms pulse
    midValue = 306+30  # 1.5ms

    
    minAngle_deg = 45
    midAngle_deg = 90
    maxAngle_deg = 155
    
    # Hence convert
    if angle_deg >= midAngle_deg:
        valuePerDegree = (maxValue-midValue)/(maxAngle_deg-midAngle_deg)
        startAngle = midAngle_deg
        start = midValue
    else:
        valuePerDegree = (midValue-minValue)/(midAngle_deg-minAngle_deg)
        startAngle = minAngle_deg
        start = midValue
        
    print("v/a", valuePerDegree)
    print("sa", startAngle)
    print("sv", start)
    value = (angle_deg-startAngle)*valuePerDegree + start
    
    return int(math.floor(value))



def positionCacheSet(posCollection):
    """
    Set the position cache for the various positions
    """
    # Dismiss all first, as some may be previously listening
    p.dismissAll()
    
    # Then set cache for each
    for pos in posCollection:
        print(pos)
        value = angleToValue(pos.angle_deg)
        print(pos.angle_deg, value)
        
        # Do listen once command for each servo
        for servoID in pos.servoIDs:
            # Set listen once for this servo
            p.sendCommand(p.LISTENONCE, protocol.Uint8_t(servoID))
        
        # Do set position cache for each servo
        p.sendCommand(p.CACHEPOS, protocol.Uint8_t(value))
    
    # Be a good neighbour and dismiss all
    p.dismissAll()



if __name__ == "__main__":
    
    Position = namedtuple("Position", ["angle_deg", "servoIDs"])
    
    pos1 = [Position(90, [0, 1, 2, 3, 4, 5, 6, 7])]
    pos2 = [Position(90+30, [0, 2, 4, 6]),
            Position(90-30, [1, 3, 5, 7])]
    positions = [pos1, pos2]

    positions = []
    o = 60
    for x in [90-o, 90, 90+o]:#range(45, 90+45, 5):
        posEven = Position(    x, list(range(0, 16, 2)))
        posOdd =  Position(180-x, list(range(1, 16, 2)))
        positions.append([posEven, posOdd])
    for x in [90-o, 90, 90+o]:#range(45, 90+45, 5):
        posEven = Position(180-x, list(range(0, 16, 2)))
        posOdd =  Position(    x, list(range(1, 16, 2)))
        positions.append([posEven, posOdd])

    for repeatIdx in range(1):
        for pos in positions:
            print("")
            print(pos)
            #input("\tSet?")
            positionCacheSet(pos)
            #input("\tExecute?")
            p.positionCacheExecute()
            time.sleep(0.5)


    