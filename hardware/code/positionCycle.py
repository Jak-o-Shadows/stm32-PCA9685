
import math
from collections import namedtuple
import time

import protocol

port = "COM8"
baud = 9600

p = protocol.RueP()
p.connect(port, baud)



def angleToValue(angle_deg: float,
                 minValue: int,
                 midValue: int,
                 maxValue: int,
                 minAngle_deg: float,
                 midAngle_deg: float,
                 maxAngle_deg: float) -> int:

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
    
    
    SimpleCalibration = namedtuple("SimpleCalibration", ["minValue", 
                                                         "midValue",
                                                         "maxValue",
                                                         "minAngle_deg",
                                                         "midAngle_deg",
                                                         "maxAngle_deg"])
    # Fill calibration data for these specific servos
    calibData = {}
    minAngle_deg = 45
    midAngle_deg = 90
    maxAngle_deg = 135
    calibData[0] = SimpleCalibration(180, 336, 282, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[1] = SimpleCalibration(180, 300, 250, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[2] = SimpleCalibration(450, 328, 274, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[3] = SimpleCalibration(180, 348, 406, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[4] = SimpleCalibration(450, 320, 270, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[5] = SimpleCalibration(180, 332, 286, minAngle_deg, midAngle_deg, maxAngle_deg)

    
    # Then set cache for each
    for pos in posCollection:
        
        # Do listen once command for each servo
        for servoID in pos.servoIDs:
            print(servoID)
            # Servo calibration done PC side for now:
            value = angleToValue(pos.angle_deg, *calibData[servoID])
            
            # Set listen once for this servo
            p.sendCommand(p.LISTENONCE, protocol.Uint8_t(servoID))
            
            # Set position cache for this servo (as calibration done PC side)
        
            # Do set position cache for each servo
            p.sendCommand(p.CACHEPOS, protocol.Uint8_t(value))
    
    # Be a good neighbour and dismiss all
    p.dismissAll()



if __name__ == "__main__":
    
    Position = namedtuple("Position", ["angle_deg", "servoIDs"])
    
    pos1 = [Position(90, [0, 1, 2, 3, 4, 5])]
    pos2 = [Position(90+30, [0, 2, 4]),
            Position(90-30, [1, 3, 5])]
    positions = [pos1, pos2]

    positions = []
    o = 50
    for x in [90, 90+o]:#range(45, 90+45, 5):
        positions.append([Position(x, [0, 1, 2, 3, 4, 5])])
        #posEven = Position(    x, list(range(0, 6, 2)))
        #posOdd =  Position(180-x, list(range(1, 6, 2)))
        #positions.append([posEven, posOdd])
    #for x in [90, 90+o]:#range(45, 90+45, 5):
    #    posEven = Position(180-x, list(range(0, 6, 2)))
    #    posOdd =  Position(    x, list(range(1, 6, 2)))
    #    positions.append([posEven, posOdd])

    for repeatIdx in range(4):
        for pos in positions:
            print("")
            print(pos)
            #input("\tSet?")
            positionCacheSet(pos)
            #input("\tExecute?")
            p.positionCacheExecute()
            time.sleep(1)


    