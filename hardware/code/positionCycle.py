
import math
from collections import namedtuple
import time
import dataclasses
from dataclasses import dataclass

import numpy as np

import protocol

port = "COM5"
baud = 9600

p = protocol.RueP()
p.connect(port, baud)

Position = namedtuple("Position", ["angle_deg", "servoIDs"])



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
        start = minValue
        
    #print("v/a", valuePerDegree)
    #print("sa", startAngle)
    #print("sv", start)
    value = (angle_deg-startAngle)*valuePerDegree + start
    #print(value)
    return int(math.floor(value))



def positionCacheSet(posCollection):
    """
    Set the position cache for the various positions
    """
    # Dismiss all first, as some may be previously listening
    p.dismissAll()
    
    @dataclass
    class SimpleCalibration:
        minValue: float
        midValue: float
        maxValue: float
        minAngle_deg: float
        midAngle_deg: float
        maxAngle_deg: float

    # Fill calibration data for these specific servos
    calibData = {}
    minAngle_deg = 40
    midAngle_deg = 90+15
    maxAngle_deg = 135
    calibData[0] = SimpleCalibration(408, 333, 210, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[1] = SimpleCalibration(436, 348, 223, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[2] = SimpleCalibration(412, 328, 220, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[3] = SimpleCalibration(256, 338, 458, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[4] = SimpleCalibration(412, 336, 217, minAngle_deg, midAngle_deg, maxAngle_deg)
    calibData[5] = SimpleCalibration(404, 340, 218, minAngle_deg, midAngle_deg, maxAngle_deg)
    
    # Only did one sided calibration
    for servoIdx, calib in calibData.items():
        value = calib.midValue + -1*(calib.midAngle_deg - calib.minAngle_deg)*(calib.maxValue-calib.midValue)/(calib.maxAngle_deg-calib.midAngle_deg)
        print("old: %f. New: %f" % (calib.minValue, value))
        calib.minValue = value

    # import matplotlib as mpl
    # import matplotlib.pyplot as plt
    # fig, ax = plt.subplots()
    # for servoIdx, calib in calibData.items():
        # x = [calib.minAngle_deg, calib.midAngle_deg, calib.maxAngle_deg]
        # y = [calib.minValue, calib.midValue, calib.maxValue]
        # label = "%d" % servoIdx
        # ax.plot(x, y, 'o-', label=label)
    # ax.legend()
    # plt.show()
    

    
    # Then set cache for each
    for pos in posCollection:
        
        # Do listen once command for each servo
        for servoID in pos.servoIDs:
            print("\n", servoID)
            # Servo calibration done PC side for now:
            value = angleToValue(pos.angle_deg, *dataclasses.astuple(calibData[servoID]))
            
            # Set listen once for this servo
            p.sendCommand(p.LISTENONCE, protocol.Uint8_t(servoID))
            
            # Set position cache for this servo (as calibration done PC side)
        
            # Do set position cache for each servo
            p.sendCommand(p.CACHEPOS, protocol.Uint8_t(value))
    
    # Be a good neighbour and dismiss all
    p.dismissAll()


def old():
    
    
    pos1 = [Position(90, [0, 1, 2, 3, 4, 5])]
    pos2 = [Position(90+30, [0, 2, 4]),
            Position(90-30, [1, 3, 5])]
    positions = [pos1, pos2]

    positions = []
    o = 50
    numSteps = 7
    legAngles = np.linspace(50, 90+o, numSteps)
    legAngles = [90, 130]
    #legAngles = np.hstack((legAngles, legAngles[::-1]))
    for x in legAngles:
        positions.append([Position(x, [0, 1, 2, 3, 4, 5])])
        #posEven = Position(    x, list(range(0, 6, 2)))
        #posOdd =  Position(180-x, list(range(1, 6, 2)))
        #positions.append([posEven, posOdd])
    #for x in [90, 90+o]:#range(45, 90+45, 5):
    #    posEven = Position(180-x, list(range(0, 6, 2)))
    #    posOdd =  Position(    x, list(range(1, 6, 2)))
    #    positions.append([posEven, posOdd])
    
    # +- 25
    posSet_angle1 = [
                     Position(66, [0]),
                     Position(103, [1]),
                     Position(109, [2]),
                     Position(91, [3]),
                     Position(83, [4]),
                     Position(64, [5])
                     ]
    posSet_angle2 = [
                     Position(103, [0]),
                     Position(66, [1]),
                     Position(64, [2]),
                     Position(83, [3]),
                     Position(91, [4]),
                     Position(109, [5])
                     ]
    for repeatCount in range(0):
        positions.append(posSet_angle1)
        positions.append(posSet_angle2)

    # Set back to neutral
    positions.append(positions[0])

    # Translate 8cm
    posSet_trans1 = [
                     Position(66, [0]),
                     Position(66, [1]),
                     Position(147, [2]),
                     Position(121, [3]),
                     Position(121, [4]),
                     Position(147, [5])
                     ]
    posSet_trans2 = [
                     Position(149, [0]),
                     Position(149, [1]),
                     Position(77, [2]),
                     Position(99, [3]),
                     Position(99, [4]),
                     Position(77, [5])
                     ]
    for repeatCount in range(3):
        positions.append(posSet_trans1)
        positions.append(posSet_trans2)
        
        
        
    #positions = []
    #positions.append([Position(105, [0, 1, 2, 3, 4, 5])])
    
    #positions = [pos1] + 3*[posSet_angle1, posSet_angle2, posSet_angle1, posSet_angle2]

    repeatSleepTime_s = 3
    interPosSleepTime_s = 1
    reverse = True

    for repeatIdx in range(1):
        for pos in positions:
            #print("")
            #print(pos)
            #input("\tSet?")
            positionCacheSet(pos)
            #input("\tExecute?")
            p.positionCacheExecute()
            #print("sleep")
            time.sleep(interPosSleepTime_s)
        time.sleep(repeatSleepTime_s)
        if reverse:
            for pos in positions[::-1]:
                #print("")
                #print(pos)
                #input("\tSet?")
                positionCacheSet(pos)
                #input("\tExecute?")
                p.positionCacheExecute()
                #print("sleep")
                time.sleep(interPosSleepTime_s)
            time.sleep(repeatSleepTime_s)


if __name__ == "__main__":

    import sys
    sys.path.append('..\\..\\..\\stewart-gough platform\\stewiegough')
    from configuration import *
    import rotary
    import fk
    
    translation = [0, 0, 0.13]  # metres
    angles = list(np.radians([0, 0, 0]))  # degrees





#    for yaw_deg in np.arange(-30, 30, 2):
#        angles[2] = np.radians(yaw_deg)
    for pitch_deg in np.arange(-25, 25, 5):
        angles[1] = np.radians(pitch_deg)


        for roll_deg in np.arange(-25, 25, 5):
            angles[0] = np.radians(roll_deg)

            upperNew = rotary.ik(np.array([0, 0, 0]), pPos, legLower, legUpper, 0, translation+angles)
            
            midJoint, leverAngles = rotary.legPos(bPos, upperNew, legLower, legUpper, legYawAngle)
            for jointIndex, pos in enumerate(midJoint):
                if pos[2] < 0:
                    leverAngles[jointIndex] *= -1
            print("Lower Leg Length", np.sqrt(np.sum(np.square(midJoint-bPos), 1)))
            print("Upper Leg Length", np.sqrt(np.sum(np.square(upperNew-midJoint), 1)))
            print("Lower Leg Angles", np.degrees(leverAngles))
            
            motorAngles = np.degrees(leverAngles) + 90
            print(motorAngles)
            
            pos = [Position(x, [i]) for i, x in enumerate(motorAngles)]
            
            positionCacheSet(pos)
            p.positionCacheExecute()

            time.sleep(0.0625)
            time.sleep(1)


    
    
    
    
    
    
    



    