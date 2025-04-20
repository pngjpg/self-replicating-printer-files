# -*- coding: utf-8 -*-
"""
Created on Thu Aug  6 17:27:54 2020

@author: Brian Minnick
"""
#===============================Parameters=====================================
#part 1
"""
Filename                Model             Xoffset   Yoffset   Scale
input                   Stanford Bunny    0         400       2
CFDMP_20mm_cube         20mm test cube    600       200       1
CFDMP_moon_city_final   moon city
SampleBenchy
"""

#part 1
filein = "CFDMP_20mm_cube.gcode"
# filein = inputs[8]
rawGCodeFile = open(str(filein), "r") # the file to be converted to a data strip

import time
startTime = time.perf_counter() # the start time of the program

#part 2
enableDoubleActivations = True # Calculate resistance when 2 channels are activated at once
enableTripleActivations = True # Calculate resistance when 3 channels are activated at once
enableQuadrupleActivations = False # Calculate resistance when 4 channels are activated at once. Not Recommended
modelCutoffEn = False #read entire GCode file or stop at cutoff value?
modelCutoff = 100000000 #lines of GCode to read before stopping. Reduces compute time for large models to position them in display window properly

#part 3
maximumSpeed = 1800 #maximum observed speed in RPM. This can include gear reduction if not accounted for in motorRotationsPerMM below
minimumSpeed = None
minimumVoltage = None #TODO: determine minimum operational voltage of the 3D printed motor
useDebugSpeedList = False #use a debug list of 1000 evenly spaced heading vectors?
maximumItt = 1000 #How many debug speeds to create

#part 4
motorRotationsPerMM = 10 # How many rotations of the motor correspond to 1mm of travel on each axis.
                         # currently, the program assumes this value is the same for all axes.
maximumSurfaceSpeed = 3 #the surface speed of the hotend in mm/s

#part 5
extruderXmmPerEmm = 21.55 #mm of X axis movement per mm of E axis movement. This only effects the actual printing of the data strip, not the encoded model itself!
slotBufferLength = 5 #in mm. this value accounts for the arm not making contact at the very start of the slot
stripFeedRate = 1 #in mm/s. the rate at which the data strip is fed through the mechanism

#part 6
import os
try:
    os.mkdir("output2024")
    os.mkdir("output/canvasImages")
    print('\n----Creating directories\'/output\' and \'/output/canvasImages\'\nTo see the program outputs, plese visit these folders.\n')
except:
    print("\n----Output directories already exist!\n")
xDrawOffset = 10 # offset within Tkinter window and PIL image when drawing paths
yDrawOffset = 100 # offset within Tkinter window and PIL image when drawing paths
drawScale = 7 # scaling factor all points are multiplied by to display. Does not effect the actual model, just the display in the TKinter window
learningRate = 0.01 # a scaling factor for gradient descent. Keeps the solution from diverging to infinity
gradientThreshold = 0.01 # the gradient value which signals the optimization has neared the minima of the cost function.
# what colors to draw to the TKinter window
drawRed = False
drawGreen = True
drawBlack = True

drawPlot = False # display the TKinter window to the screen?

#part 7
writeOutput = True #write GCode (data strip model) to the output file?

#part 8

#part 9
processVideo = False # should the error graphs and canvas images be generated and saved?
videoDivideFactor = 1 # how many entries to skip when plotting error to minimize video length. Makes the video shorter for long models.
plotdpi = 200 # resolution of the plot
errorPlotFilenameA = "output/animatedErrorA.mp4" # filename of the animated error After gradient descent
errorPlotFilenameB = "output/animatedErrorB.mp4" # filename of the animated error Before gradient descent
canvasImagesRootFolder = "output/canvasImages/" # root directory for canvas images

#part 10
reconstructGCode = False # Reconstruct the GCode into a viewable model?

#=================================Part 1=======================================
print("Begin Part 1: Read GCode file")

#First, the GCode file is opened and looped through to detect important lines
absXPoints = [0] # the desired X Coordinate of the hotend
absYPoints = [0] # the desired Y Coordinate of the hotend
absZPoints = [0] # the desired Z Coordinate of the hotend
extruderActive = [] # should the extruder be extruding? Since the feedrate is constant, the volumetric flowrate through the nozzle is constant so an extruder speed is not needed.
GCodeLineCouter = 0

for line in rawGCodeFile:
    if modelCutoffEn and GCodeLineCouter > modelCutoff:
        print("Ending GCode read at %f because modelCutoffEn is true" % modelCutoff)
        break
    else:
        if((line[:2] == "G1")or(line[:2] == "G0")):
            temporarySplitLine = line.split(" ")
            xHit = False # used to ensure each G1 or G0 line is give a full XYZ point even if a specific word doesn't exist
            yHit = False
            zHit = False
            eHit = False
            for word in temporarySplitLine: # A word is each space separated element in a GCode line. G1 X10 has one word, X10.
                if (word[:1] == ";"):
                    break
                elif (word[:1] == "X"):
                    absXPoints.append(word[1:])
                    xHit = True
                elif (word[:1] == "Y"):
                    absYPoints.append(word[1:])
                    yHit = True
                elif (word[:1] == "Z"):
                    absZPoints.append(word[1:])
                    zHit = True
                elif (word[:1] == "E") and (xHit or yHit or zHit): # is the extruder supposed to be running and is there an X, Y, or Z component to the line. Extrusion only lines are not currently supported
                    if (word[:2] == "E-"):
                        extruderActive.append(-1)
                    else:
                        extruderActive.append(1)
                    eHit = True
            if not xHit and not yHit and not zHit: # if there is no movement on the X, Y, or Z axes, the line is an extrusion only move and must be removed. 
                print("extrusion only line detected and removed:\n    \'" + str(line))
            else:
                if not xHit: # if the movement only includes some of the axes, set the unmoved axes to its last position.
                    absXPoints.append(absXPoints[len(absXPoints)-1])
                if not yHit:
                    absYPoints.append(absYPoints[len(absYPoints)-1])
                if not zHit:
                    absZPoints.append(absZPoints[len(absZPoints)-1])
                if not eHit:
                    extruderActive.append(0)
                GCodeLineCouter += 1

print("End Part 1. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 2=======================================
print("Begin Part 2: Calculate Currents")

#Now, the output currents of all possible combinations of potentiometer will be calculated.
import numpy as np
from itertools import combinations
from collections import OrderedDict
import math

inputVoltage = 30 #vi
motorResistance = 7.8
cutSwitchResistance = 8 #The cut switch adds a resistor in parallel with the motor to cur the current in half, This is to even out the distribution of speeds
#potResistances = [27.12, 20.63, 12.8, 12.36, 12.37, 2.19]
potResistances = [52.96, 36.81, 36.6, 21.67, 4.56, 7.51] # optimized

R1list = []
R2list = []

#      Controller         Motor 
# +Vi -----|==|-----o-----|==|--- +0
#                   |Vmot

#keys are the resulting voltage, values are the combination of activated channels that result in that voltage
voltages = {'0': [0,0,0,0,0,0,0]}
currents = {'0': [0,0,0,0,0,0,0]}

#single activation voltages
for positions in combinations(range(6), 1):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        R1 = (1/((1/Rlist[0]))) #the resistance of all activated channels are added using the parallel resistance equation
        current = inputVoltage/(R1+motorResistance)
        currents[current] = p 
        voltage = inputVoltage-(inputVoltage*(R1/(R1+motorResistance)))
        voltages[voltage] = p

for positions in combinations(range(6), 1):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        p[6] = 1
        R1 = (1/((1/Rlist[0]))) #the resistance of all activated channels are added using the parallel resistance equation
        R2 = (1/((1/motorResistance)+(1/cutSwitchResistance))) #the resistance of the motor and bypass resistor in parallel is calculated
        current = inputVoltage/(R1+R2)
        currents[current*(motorResistance/(motorResistance+cutSwitchResistance))] = p #The current is cut in half (roughly) by the bypass resistor
        voltage = inputVoltage-(inputVoltage*(R1/(R1+R2))) #the voltage drop across the first resistors is calculated.
        voltages[voltage*(motorResistance/(motorResistance+cutSwitchResistance))] = p #Since the current is cut in half due to the bypass resistor in parallel, the "apparent" voltage to achieve the same current without the cut switch is half.

#Controller__________  
#        v          v 
#              r1       Motor     
#        o----|==|---o---|==|---- +0v
# +Vi ---|     r2    |
#        o----|==|---o
#                    |Vmot

if enableDoubleActivations:
    for positions in combinations(range(6), 2):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        R1 = (1/((1/Rlist[0])+(1/Rlist[1])))
        current = inputVoltage/(R1+motorResistance)
        currents[current] = p 
        voltage = inputVoltage-(inputVoltage*(R1/(R1+motorResistance)))
        voltages[voltage] = p
        
    for positions in combinations(range(6), 2):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        p[6] = 1
        R1 = (1/((1/Rlist[0])+(1/Rlist[1]))) #the resistance of all activated channels are added using the parallel resistance equation
        R2 = (1/((1/motorResistance)+(1/cutSwitchResistance))) #the resistance of the motor and cutswitch in parallel is calculated
        current = inputVoltage/(R1+R2)
        currents[current*(motorResistance/(motorResistance+cutSwitchResistance))] = p #The current is cut in half (roughly) by the bypass resistor
        voltage = inputVoltage-(inputVoltage*(R1/(R1+R2))) #the voltage drop across the first resistors is calculated.
        voltages[voltage*(motorResistance/(motorResistance+cutSwitchResistance))] = p #Since the current is cut in half due to the cut switch in parallel, the "apparent" voltage to achieve the same current without the cut switch is half.

#Controller__________  
#        v          v 
#              r1       Motor     
#        o----|==|---o---|==|---- +0v
# +Vi ---|     r2    | 
#        o----|==|---o
#        |     r3    |
#        o----|==|---o
#                    |Vmot

#triple activation voltages              
if enableTripleActivations:
    for positions in combinations(range(6), 3):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        R1 = (1/((1/Rlist[0])+(1/Rlist[1])+(1/Rlist[2])))
        current = inputVoltage/(R1+motorResistance)
        currents[current] = p 
        voltage = inputVoltage-(inputVoltage*(R1/(R1+motorResistance)))
        voltages[voltage] = p
        
    for positions in combinations(range(6), 3):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        p[6] = 1
        R1 = (1/((1/Rlist[0])+(1/Rlist[1])+(1/Rlist[2]))) #the resistance of all activated channels are added using the parallel resistance equation
        R2 = (1/((1/motorResistance)+(1/cutSwitchResistance))) #the resistance of the motor and cutswitch in parallel is calculated
        current = inputVoltage/(R1+R2)
        currents[current*(motorResistance/(motorResistance+cutSwitchResistance))] = p #The current is cut in half (roughly) by the bypass resistor
        voltage = inputVoltage-(inputVoltage*(R1/(R1+R2))) #the voltage drop across the first resistors is calculated.
        voltages[voltage*(motorResistance/(motorResistance+cutSwitchResistance))] = p #Since the current is cut in half due to the cut switch in parallel, the "apparent" voltage to achieve the same current without the cut switch is half.
        
#quadruple activation voltages
if enableQuadrupleActivations:
    for positions in combinations(range(6), 4):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        R1 = (1/((1/Rlist[0])+(1/Rlist[1])+(1/Rlist[2])+(1/Rlist[3])))
        current = inputVoltage/(R1+motorResistance)
        currents[current] = p 
        voltage = inputVoltage-(inputVoltage*(R1/(R1+motorResistance)))
        voltages[voltage] = p
        
    for positions in combinations(range(6), 4):
        p = [0] * 7
        Rlist = []
        for i in positions:
            p[i] = 1
            Rlist.append(potResistances[i])
        p[6] = 1
        R1 = (1/((1/Rlist[0])+(1/Rlist[1])+(1/Rlist[2])+(1/Rlist[3]))) #the resistance of all activated channels are added using the parallel resistance equation
        R2 = (1/((1/motorResistance)+(1/cutSwitchResistance))) #the resistance of the motor and cutswitch in parallel is calculated
        current = inputVoltage/(R1+R2)
        currents[current*(motorResistance/(motorResistance+cutSwitchResistance))] = p #The current is cut in half (roughly) by the bypass resistor
        voltage = inputVoltage-(inputVoltage*(R1/(R1+R2))) #the voltage drop across the first resistors is calculated.
        voltages[voltage*(motorResistance/(motorResistance+cutSwitchResistance))] = p #Since the current is cut in half due to the cut switch in parallel, the "apparent" voltage to achieve the same current without the cut switch is half.



     
#voltagesSorted = OrderedDict(sorted(voltages.items(), key=lambda x:x[1], reverse=True))

print("End Part 2. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 3=======================================
print("Begin Part 3: Estimate Speeds")

#We must determine the rotation speed of the motor at a given voltage
speeds = {0.0: [0,0,0,0,0,0,0]}
maximumSpeed = 1800 #maximum observed speed in RPM. This can include gear reduction
minimumSpeed = None
minimumMotorCurrent = min(currents, key=float) #TODO: determine minimum operational voltage
maximumMotorCurrent = max(currents, key=float)

if not useDebugSpeedList:
    for key in currents:
        IRatio = float(key)/maximumMotorCurrent
        rotationSpeed = ((IRatio*maximumSpeed)*(1/motorRotationsPerMM))/60 #mm/s
        speeds[rotationSpeed] = currents[key]
    speedsSorted = OrderedDict(sorted(speeds.items(), key=lambda x:x[1], reverse=True))
else:
    for x in range(maximumItt): #use evenly spaced speeds for testing purposes
        rotationSpeed = maximumSurfaceSpeed*((x+1)/maximumItt)
        speeds[rotationSpeed] = [1]
    speedsSorted = OrderedDict(sorted(speeds.items(), key=lambda x:x[1], reverse=True))

print("End Part 3. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 4=======================================
print("Begin Part 4: Find Closest Speeds")

#Now using the rotation speeds and the points from the GCode file, the duration of each movement can be calculated.
#along with that, the voltage on each motor is calculated and the corresponding channels are selected
#import bisect 

movementDurations = list()
closestSpeeds = list(list())
closestActivatedchannels = list(list())
requestedSpeeds = list(list())

for index in range(len(absXPoints)):
    if index == 0:
        pass
    else:
        try:
            previousXPos = float(absXPoints[index-1])
            previousYPos = float(absYPoints[index-1])
            previousZPos = float(absZPoints[index-1])
            currentXPos = float(absXPoints[index])
            currentYPos = float(absYPoints[index])
            currentZPos = float(absZPoints[index])
            deltaX = currentXPos - previousXPos
            deltaY = currentYPos - previousYPos
            deltaZ = currentZPos - previousZPos
            deltaC = math.sqrt((pow(deltaX, 2)+(pow(deltaY, 2))+(pow(deltaZ, 2))))
            movementDuration = deltaC/maximumSurfaceSpeed #distance/velocity = time
            movementDurations.append(movementDuration) #movement duration in seconds
        except:
            print("!first try-except broken in part 4")
            break
        try:
            XReqSpeed = (deltaX/movementDuration)
            YReqSpeed = (deltaY/movementDuration)
            ZReqSpeed = (deltaZ/movementDuration)
            
            # Activate if using Binary Search as implemented below. This is not optimal.
            # XReqSpeed = (deltaX/movementDuration)*0.999 #this multiple is needed or else the binary search can return a value not in the speed list
            # YReqSpeed = (deltaY/movementDuration)*0.999
            # ZReqSpeed = (deltaZ/movementDuration)*0.999
            
            surfaceSpeed = (pow(( pow(XReqSpeed,2) + pow(YReqSpeed,2) + pow(ZReqSpeed,2)),1/2))  
            assert surfaceSpeed < 3.005 and surfaceSpeed > 2.995
            
            minimumDifference = 1000
            closestSpeedX = -1000
            for key in speeds: #TODO: This is horribly inefficient, someone fix me! Binary search?
                difference = abs(abs(XReqSpeed) - float(key))
                if difference < minimumDifference:
                    minimumDifference = difference
                    closestSpeedX = key
            
            
            minimumDifference = 1000
            closestSpeedY = -1000
            for key in speeds: #TODO: This is horribly inefficient, someone fix me! Binary search?
                difference = abs(abs(YReqSpeed) - float(key))
                if difference < minimumDifference:
                    minimumDifference = difference
                    closestSpeedY = key
            
            minimumDifference = 1000
            closestSpeedZ = -1000
            for key in speeds: #TODO: This is horribly inefficient, someone fix me! Binary search?
                difference = abs(abs(ZReqSpeed) - float(key))
                if difference < minimumDifference:
                    minimumDifference = difference
                    closestSpeedZ = key
            
            if closestSpeedX == -1000:
                closestSpeedX = 0
            if closestSpeedY == -1000:
                closestSpeedY = 0
            if closestSpeedZ == -1000:
                closestSpeedZ = 0
            
            # closestSpeedX = min(speeds.keys(), key = lambda key: abs(key-XReqSpeed))
            # closestSpeedY = min(speeds.keys(), key = lambda key: abs(key-YReqSpeed))
            # closestSpeedZ = min(speeds.keys(), key = lambda key: abs(key-ZReqSpeed))
            
            # closestSpeedIndexX = bisect.bisect_left(list(speedsSorted.keys()), abs(XReqSpeed))
            # closestSpeedIndexY = bisect.bisect_left(list(speedsSorted.keys()), abs(YReqSpeed))
            # closestSpeedIndexZ = bisect.bisect_left(list(speedsSorted.keys()), abs(ZReqSpeed))
            # speedsList = list(speedsSorted.keys())
            # closestSpeedX = speedsList[closestSpeedIndexX]
            # closestSpeedY = speedsList[closestSpeedIndexY]
            # closestSpeedZ = speedsList[closestSpeedIndexZ]
                
            requestedSpeeds.append([XReqSpeed, YReqSpeed, ZReqSpeed])
            closestSpeeds.append([closestSpeedX, closestSpeedY, closestSpeedZ])
            closestActivatedchannels.append([speeds[closestSpeedX], speeds[closestSpeedY], speeds[closestSpeedZ]])
        except:
            print("ERROR: Extrusion only move detected at line %i. This usually occurs if the program is stopped in the middle of part 4" % index)
            # absXPoints.pop(index-1)
            # absYPoints.pop(index-1)
            # absZPoints.pop(index-1)
            # extruderActive.pop(index-1)
            
print("End Part 4. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 5=======================================
print("Begin Part 5: Compile Activated Channels")

#now, the data strip is compiled into one big list.
currentXPosition = 10
currentYPosition = 10
currentEPosition = 0
fullDataStripSegment = []
completeDataStrip = list(list())

for index, channelSet in enumerate(closestActivatedchannels):
    fullDataStripSegment = []
    for channel in channelSet[0]:
        fullDataStripSegment.append(channel)
    for channel in channelSet[1]:
        fullDataStripSegment.append(channel)
    for channel in channelSet[2]:
        fullDataStripSegment.append(channel)
    
    if(extruderActive[index] < 0):
        fullDataStripSegment.append(1)
        fullDataStripSegment.append(0)
        fullDataStripSegment.append(0)
        fullDataStripSegment.append(1)
    if(extruderActive[index] > 0):
        fullDataStripSegment.append(0)
        fullDataStripSegment.append(1)
        fullDataStripSegment.append(1)
        fullDataStripSegment.append(0)
    elif(extruderActive[index] == 0):
        fullDataStripSegment.append(0)
        fullDataStripSegment.append(0)
        fullDataStripSegment.append(0)
        fullDataStripSegment.append(0)    
        
    for x in range(3):
        if(closestSpeeds[index][2-x] == 0):
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(0)
        elif(requestedSpeeds[index][2-x] < 0):
            fullDataStripSegment.append(1)
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(1)
        elif(requestedSpeeds[index][2-x] > 0):
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(1)
            fullDataStripSegment.append(1)
            fullDataStripSegment.append(0)
        elif(requestedSpeeds[index][2-x] == 0):
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(0)
            fullDataStripSegment.append(0)
        else:
            raise Exception("Error, this point should not be reached")
    
    completeDataStrip.append(fullDataStripSegment)
print("End Part 5. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 6=======================================
print("Begin Part 6: Optimize Movement Durations With Gradient Descent")

#the propagated data strip coordinates are compared to the ground truth file
#the ground truth and propagated coordinates are displayed.
#a custom gradient descent algorithm is applied to minimize error in the data strip
import tkinter as tk
from PIL import Image, ImageDraw
import sys

propagatedCoordinates = [0,0,0]
currentPos = [0,0,0]
oldPropagatedCoordinates = [0,0,0]
oldAbsoluteCoordinates = [0,0,0]
oldAfterGradientDescentPropagatedCoordinates = [0,0,0]
afterGradientDescentPropagatedCoordinates = [0,0,0]
summedErrorAGD = [0,0,0]
summedErrorBGD = [0,0,0]
agdCoordinateList = list(list())
bgdCoordinateList = list(list())
updatedMovementDurations = list()
BGDErrorHistory = list()
AGDErrorHistory = list()
window = tk.Tk()
canvas = tk.Canvas(window,width=800,height=800)

def getCurrentAbsolutePosition (index):
    coordinate = list()
    coordinate.append(float(absXPoints[index]))
    coordinate.append(float(absYPoints[index]))
    coordinate.append(float(absZPoints[index]))
    return coordinate
def subtractLists (l1, l2):
    l1 = [float(item) for item in l1]
    l2 = [float(item) for item in l2]
    output = list()
    for i in range(len(l1)):
        output.append(l1[i]-l2[i])
    return output
def addLists (l1, l2):
    output = list()
    for i in range(len(l1)):
        output.append(l1[i]+l2[i])
    return output
def divideList (l1, div):
    output = list()
    for i in range(len(l1)):
        output.append(l1[i]/div)
    return output
def squareList (l1):
    output = list()
    for i in range(len(l1)):
        output.append(pow(l1[i],2))
    return output
def rootList (l1):
    output = list()
    for i in l1:
        output.append(pow(l1[i], 0.5))
def calculateError(currentPos, propagatedCoordinates):
    delta = subtractLists(currentPos, propagatedCoordinates)
    squaredError = squareList(delta)
    return squaredError
def findVideoFrameID (index): #a recursive function that ensures the video frame ID has the proper number of characters
    if not len(index) == 7:
        index2 = str("0" + index)
        return findVideoFrameID(index2)
    else:
        return index

#TODO: if movement duration is 0 or less than 1, change the motor direction

# a PIL image in memory for animation purposes
image = Image.new("RGB", (800, 800), (255, 255, 255)) #create a new image with dimensions 800x800 and a white background
draw = ImageDraw.Draw(image)

for index, closestSpeedList in enumerate(closestSpeeds):
    # calculate the toolhead position from the data strip
    
    # save the old coordinates
    oldPropagatedCoordinates = propagatedCoordinates
    oldAfterGradientDescentPropagatedCoordinates = afterGradientDescentPropagatedCoordinates
    
    # calculate the offset from the old coordinates
    offsetComponents = list()
    for x in range(3):
        if requestedSpeeds[index][x] < 0:
            offsetComponents.append((-1)*(float(closestSpeedList[x])*movementDurations[index]))
        else:
            offsetComponents.append(float(closestSpeedList[x])*movementDurations[index])
    propagatedCoordinates = addLists(propagatedCoordinates, offsetComponents)
    currentPos = getCurrentAbsolutePosition(index) #the ground truth coordinate
    
    # a try catch statement to catch unwanted index out of bounds errors, this should not trip
    # this is needed to break the for loop above so errors do not pile up and the stack trace is more readable
    try:
        # draw the next lines to the canvas and save the current image to the image root folder for animation
        nextPos = getCurrentAbsolutePosition(index+1)
        if drawRed:
            canvas.create_line((xDrawOffset+oldPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*oldPropagatedCoordinates[1]))*drawScale, (xDrawOffset+propagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*propagatedCoordinates[1]))*drawScale, fill = 'red')
            draw.line([(xDrawOffset+oldPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*oldPropagatedCoordinates[1]))*drawScale, (xDrawOffset+propagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*propagatedCoordinates[1]))*drawScale], (255, 161, 39))
        if drawBlack:
            canvas.create_line((xDrawOffset+currentPos[0])*drawScale, (yDrawOffset+(-1*currentPos[1]))*drawScale, (xDrawOffset+nextPos[0])*drawScale, (yDrawOffset+(-1*nextPos[1]))*drawScale)
            draw.line([(xDrawOffset+currentPos[0])*drawScale, (yDrawOffset+(-1*currentPos[1]))*drawScale, (xDrawOffset+nextPos[0])*drawScale, (yDrawOffset+(-1*nextPos[1]))*drawScale], (232, 190, 39))
        
        # calculate error
        beforeGradientDescentError = calculateError(nextPos, propagatedCoordinates)
        summedErrorBGD = addLists(summedErrorBGD, beforeGradientDescentError)
        
        # correct for this error and regenerate movement duration list using gradient descent
        
        gradient = 10
        newMovementDuration = movementDurations[index]
        counter = 0 #ensure that the correction loop is finite. Normally, the gradient reaches the cutoff threshold before the counter reaches 100
        while abs(gradient) > gradientThreshold and counter < 100:
            # gradient = 2*((-1*nextPos[1]*requestedSpeeds[index][1])-(nextPos[0]*requestedSpeeds[index][0])+(oldPropagatedCoordinates[1]*requestedSpeeds[index][1])+(oldPropagatedCoordinates[0]*requestedSpeeds[index][0])+(pow(requestedSpeeds[index][1],2)*newMovementDuration)+(pow(requestedSpeeds[index][0],2)*newMovementDuration))
            
            #2D Gradient
            #gradient = (((-2*requestedSpeeds[index][0])*(nextPos[0] - (oldAfterGradientDescentPropagatedCoordinates[0] + (requestedSpeeds[index][0]*newMovementDuration))))-((2*requestedSpeeds[index][1])*(nextPos[1] - (oldAfterGradientDescentPropagatedCoordinates[1] + (requestedSpeeds[index][1]*newMovementDuration))))) 
            #3D Gradient
            gradient = (((-2*requestedSpeeds[index][0])*(nextPos[0] - (oldAfterGradientDescentPropagatedCoordinates[0] + (requestedSpeeds[index][0]*newMovementDuration))))-((2*requestedSpeeds[index][1])*(nextPos[1] - (oldAfterGradientDescentPropagatedCoordinates[1] + (requestedSpeeds[index][1]*newMovementDuration))))-((2*requestedSpeeds[index][2])*(nextPos[2] - (oldAfterGradientDescentPropagatedCoordinates[2] + (requestedSpeeds[index][2]*newMovementDuration))))) 
            
            newMovementDuration -= learningRate*gradient
            if index == len(closestSpeeds)-1:
                if counter == 0:
                    print("\nExample Gradient Descent Parameters at " + str(index)) # Print the parameters for the gradient and cost functions at the last index. This is for external verification
                    print("X Axis Speed               :"+str(requestedSpeeds[index][0]))
                    print("True Next Position X       :"+str(nextPos[0]))
                    print("Propagated Last Position X :"+str(oldAfterGradientDescentPropagatedCoordinates[0]))
                    print("X Axis Speed               :"+str(requestedSpeeds[index][1]))
                    print("True Next Position Y       :"+str(nextPos[1]))
                    print("Propagated Last Position Y :"+str(oldAfterGradientDescentPropagatedCoordinates[1]) + "\n")
                print("Updated Movement Duration Example at index: " + str(index) + " is: " + str(newMovementDuration)) #Each time the MD is updated, print it. For external verification only
                #print(gradient)
                
            counter +=1
        
        # replace movement duration within list with the optimized value
        updatedMovementDurations.append(newMovementDuration)
        
        # recalculate movement with the new movement duration
        offsetComponents = list()
        for x in range(3):
            if requestedSpeeds[index][x] < 0:
                offsetComponents.append((-1)*(float(closestSpeedList[x])*updatedMovementDurations[index]))
            else:
                offsetComponents.append(float(closestSpeedList[x])*updatedMovementDurations[index])
        
        # draw optimized movement to screen in green
        afterGradientDescentPropagatedCoordinates = addLists(afterGradientDescentPropagatedCoordinates, offsetComponents)
        if drawGreen:
            canvas.create_line((xDrawOffset+oldAfterGradientDescentPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*oldAfterGradientDescentPropagatedCoordinates[1]))*drawScale, (xDrawOffset+afterGradientDescentPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*afterGradientDescentPropagatedCoordinates[1]))*drawScale, fill = 'green')
            draw.line([(xDrawOffset+oldAfterGradientDescentPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*oldAfterGradientDescentPropagatedCoordinates[1]))*drawScale, (xDrawOffset+afterGradientDescentPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*afterGradientDescentPropagatedCoordinates[1]))*drawScale], (154, 255, 39))
        
        # calculate error after gradient descent applied
        afterGradientDescentError = calculateError(nextPos, afterGradientDescentPropagatedCoordinates)
        summedErrorAGD = addLists(summedErrorAGD, afterGradientDescentError)
        oldAbsoluteCoordinates = currentPos
        
        agdCoordinateList.append(afterGradientDescentPropagatedCoordinates)
        bgdCoordinateList.append(propagatedCoordinates)
        
        # calculate the mean squared error of this particular point
        BGDErrorHistory.append(beforeGradientDescentError)
        AGDErrorHistory.append(afterGradientDescentError)
        
        # save the current PIL image to the image root folder
        if index%videoDivideFactor == 0 and processVideo:
            videoFrameID = findVideoFrameID(str(int(index/videoDivideFactor))) # call the recursive function which adds 0s to the beginning of the number until the correct length is achieved
            filename = canvasImagesRootFolder + "img" + str(videoFrameID) + ".png"
            #print(str(index/videoDivideFactor))
            image.save(filename)
        
        #cover up the highlighted line
        if drawRed:
            draw.line([(xDrawOffset+oldPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*oldPropagatedCoordinates[1]))*drawScale, (xDrawOffset+propagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*propagatedCoordinates[1]))*drawScale], (255, 0, 0))
        if drawBlack:
            draw.line([(xDrawOffset+currentPos[0])*drawScale, (yDrawOffset+(-1*currentPos[1]))*drawScale, (xDrawOffset+nextPos[0])*drawScale, (yDrawOffset+(-1*nextPos[1]))*drawScale], (0,0,0))
        if drawGreen:
            draw.line([(xDrawOffset+oldAfterGradientDescentPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*oldAfterGradientDescentPropagatedCoordinates[1]))*drawScale, (xDrawOffset+afterGradientDescentPropagatedCoordinates[0])*drawScale, (yDrawOffset+(-1*afterGradientDescentPropagatedCoordinates[1]))*drawScale], (0,128,0))
        
    except:
        print(sys.exc_info()[0])
        break
    
cumulativeErrorBGD = subtractLists(getCurrentAbsolutePosition(len(closestSpeeds)), propagatedCoordinates) 
MSErrorBGD = divideList(summedErrorBGD, len(closestSpeeds)+1)       
cumulativeErrorAGD = subtractLists(getCurrentAbsolutePosition(len(closestSpeeds)), afterGradientDescentPropagatedCoordinates)
MSErrorAGD = divideList(summedErrorAGD, len(closestSpeeds)+1)
print("----Before Gradient Descent (RED) Propagated Points")
print("    Mean Squared Error (X): %f" % MSErrorBGD[0])
print("    Mean Squared Error (Y): %f" % MSErrorBGD[1])
print("    Mean Squared Error (Z): %f" % MSErrorBGD[2])
# print("    Mean Squared Error (Z): %f" % pow(MSErrorBGD[2],0.5))
print("    Cumulative Error (X): %fmm" % cumulativeErrorBGD[0])
print("    Cumulative Error (Y): %fmm" % cumulativeErrorBGD[1])
print("    Cumulative Error (Z): %fmm" % cumulativeErrorBGD[2])
# print("    Cumulative Error (Z): %fmm" % cumulativeErrorBGD[2])
print("\n----After Gradient Descent (GREEN) Propagated Points")
print("    Mean Squared Error (X): %f" % MSErrorAGD[0])
print("    Mean Squared Error (Y): %f" % MSErrorAGD[1])
print("    Mean Squared Error (Z): %f" % MSErrorAGD[2])
# print("    Mean Squared Error (Z): %f" % pow(MSErrorAGD[2],0.5))
print("    Cumulative Error (X): %fmm" % cumulativeErrorAGD[0])
print("    Cumulative Error (Y): %fmm" % cumulativeErrorAGD[1])
print("    Cumulative Error (Z): %fmm" % cumulativeErrorAGD[2])
# print("    Cumulative Error (Z): %fmm" % cumulativeErrorAGD[2])
print("End Part 6. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 7=======================================
print("Begin Part 7: Write the Output to GCode File")

if writeOutput:
    #write gcode to file
    outputFile = open("output/" + filein[-10:-4].strip(".") + ".gcode", "a+")
    outputFile.seek(0)
    outputFile.truncate()
    outputFile.write(";GCode file generated by SlicerPy 2020 written by Brian Minnick\n")
    outputFile.write("M104 S190\n")               #set temp
    outputFile.write("M109 S190\n")               #wait for temp
    outputFile.write("M82\n")                     #absolute mode
    outputFile.write("M201 X100 Y100\n")          #set maximum axial acceleration
    outputFile.write("G28 X Z\n")                 #home
    outputFile.write("G92 E0\n")                  #set extruder position to 0
    outputFile.write("G1 F3600 E-6.5\n")          #retract filament
    outputFile.write("G0 F750 X10 Y10 Z0.3\n")    #move to starting location
    outputFile.write("M204 S100\n")               #set maximum acceleration
    outputFile.write("G1 F2400 E0\n")             #undo retract filament

    for index, row in enumerate(completeDataStrip):
        extrudeAmountX = 185.5/extruderXmmPerEmm
        currentEPosition += extrudeAmountX
        outputFile.write(str("G1 X195.5 E%f\n" % (currentEPosition)))
        totalSlotHeight = (slotBufferLength)+(stripFeedRate*movementDurations[index])
        extrudeAmountY = totalSlotHeight/extruderXmmPerEmm
        currentXPosition = 195.5
        for channelIndex, channel in enumerate(reversed(row)):
            if channel == 0:
                currentXPosition -=1.75
                currentEPosition += 0.08
                outputFile.write("G0 X%f E%f\n" % (currentXPosition, currentEPosition))
                currentYPosition += totalSlotHeight
                currentEPosition += extrudeAmountY
                outputFile.write("G1 Y%f E%f\n" % (currentYPosition, currentEPosition))
                currentXPosition -=0.5
                outputFile.write("G0 X%f\n" % (currentXPosition))
                currentYPosition -= totalSlotHeight
                currentEPosition += extrudeAmountY
                outputFile.write("G1 Y%f E%f\n" % (currentYPosition, currentEPosition))
                currentXPosition -=0.5
                outputFile.write("G0 X%f\n" % (currentXPosition))
                currentYPosition += totalSlotHeight
                currentEPosition += extrudeAmountY
                outputFile.write("G1 Y%f E%f\n" % (currentYPosition, currentEPosition))
                currentXPosition -=0.5
                outputFile.write("G0 X%f\n" % (currentXPosition))
                currentYPosition -= totalSlotHeight
                currentEPosition += extrudeAmountY
                outputFile.write("G1 Y%f E%f\n" % (currentYPosition, currentEPosition))
                currentXPosition -=1.75
                currentEPosition += 0.08
                outputFile.write("G0 X%f E%f ;end of channel value 0\n" % (currentXPosition, currentEPosition))
            if channel == 1:
                currentXPosition -=0.3
                outputFile.write("G0 X%f ;end of channel value 0\n" % (currentXPosition))
                currentYPosition += totalSlotHeight
                currentEPosition += extrudeAmountY
                outputFile.write("G1 Y%f E%f\n" % (currentYPosition, currentEPosition))
                currentXPosition -=4.4
                currentEPosition += 0.21
                outputFile.write("G0 X%f E%f\n" % (currentXPosition, currentEPosition))
                currentYPosition -= totalSlotHeight
                currentEPosition += extrudeAmountY
                outputFile.write("G1 Y%f E%f\n" % (currentYPosition, currentEPosition))
                currentXPosition -=0.3
                outputFile.write("G0 X%f ;end of channel value 0\n" % (currentXPosition))
        currentYPosition += totalSlotHeight
        currentXPosition = 10
        outputFile.write("G0 Y%f X%f ;end of row\n" % (currentYPosition, currentXPosition))
    outputFile.write("G0 Z10\n")
    outputFile.write("G28 X\n")#home the X axis only (no Y axis limits!)
    outputFile.close()
    print("     Total Data Stip Length: %imm" % currentYPosition)
else:
    print("GCode not written to output file because writeOutput is false!")
print("End Part 7. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 8=======================================
print("Begin Part 8: Plot Histogram and Show Frame")

#Plot graphs and pack/display canvas

#plot a histogram of all possible speeds which can be generated by the motor controller

#this histogram is the same every time the program is run, it is only necessary when comparing different resistance combinations

import matplotlib.pylab as plt
# x = list()
# for key in speeds: x.append(key)
# plt.axes(ylim =(0, 20))
# plt.hist(x, bins=10)
# plt.title("Histogram of Speeds (%i total)" % len(speeds))
# plt.xlabel("Motor Speed (mm/s)")
# plt.ylabel("Frequency")
# plt.show()

print("End Part 8. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 9=======================================
print("Begin Part 9: Animate Error")

#Animate the error graph and the TKinter canvas
import matplotlib.animation as animation

if processVideo:
    print('Begin saving animated error plot to ' + errorPlotFilenameA + " and " + errorPlotFilenameB)
    
    #calculate the vectorized error at each point in the GCode file
    vectorizedBGDError = list()
    for point in BGDErrorHistory:
        BGDOutput = pow( (pow(point[0],2) + pow(point[1],2) + pow(point[2],2)) ,0.5)
        vectorizedBGDError.append(BGDOutput)
    vectorizedAGDError = list()
    for point in AGDErrorHistory:
        AGDOutput = pow( (pow(point[0],2) + pow(point[1],2) + pow(point[2],2)) ,0.5)
        vectorizedAGDError.append(AGDOutput)
    
    #select every "videoDivideFactor"th value from error lists and remake the list
    tempListA, tempListB = list(), list()
    for index in range(len(vectorizedAGDError)):
        if index % videoDivideFactor == 0:
            tempListA.append(vectorizedAGDError[index])
            tempListB.append(vectorizedBGDError[index])
    vectorizedAGDError = tempListA
    vectorizedBGDError = tempListB
    
    figB = plt.figure(dpi=plotdpi) 
    axisB = plt.axes(xlim =(0, len(vectorizedBGDError)), ylim =(0, max(vectorizedBGDError)), title = 'Before Gradient Descent Error')
    
    figA = plt.figure(dpi=plotdpi) 
    axisA = plt.axes(xlim =(0, len(vectorizedBGDError)), ylim =(0, max(vectorizedBGDError)), title = 'After Gradient Descent Error')
    
    lines = []
    line = axisB.plot([], [], lw = 3, color = 'r')[0]
    lines.append(line)
    line = axisA.plot([], [], lw = 3, color = 'g')[0]
    lines.append(line)
    
    def init():  
        for line in lines:
            line.set_data([],[])
        return lines
       
    def animate(i): 
        lines[0].set_data(np.linspace(0,i,i), vectorizedBGDError[:i])
        lines[1].set_data(np.linspace(0,i,i), vectorizedAGDError[:i])
        
        return lines
       
    animA = animation.FuncAnimation(figA, animate, init_func = init, frames = len(vectorizedBGDError), interval = 33.3, blit = True) 
    animB = animation.FuncAnimation(figB, animate, init_func = init, frames = len(vectorizedBGDError), interval = 33.3, blit = True) 
       
    animA.save(errorPlotFilenameA, writer = 'ffmpeg', fps = 30) 
    animB.save(errorPlotFilenameB, writer = 'ffmpeg', fps = 30) 
    print('Done saving animation of error graph to ' + errorPlotFilenameA + " and " + errorPlotFilenameB)
else:
    print("Will not save animation of error because processVideo is False!")
#end animation of error plot

#animate the PIL images using the ffmpeg command below.
#an example of this and the animated error graphs can also be found here: https://youtu.be/DmkQBWcAODQ

#ffmpeg command to convert images to video:
#ffmpeg -s 800x800 -pix_fmt rgba -r 30 -i img%05d.0.png -vcodec h264 -pix_fmt yuv420p -y out.mp4
    
print("End Part 9. Completed in " + str(time.perf_counter()-startTime) + "s\n")
startTime = time.perf_counter()
#=================================Part 10======================================
print("Begin Part 10: Reconstruct the Model Encoded in the Data Strip")

#reconstruct the propagated coordinates into a GCode model which can be displayed in 3D in a slicer such as Cura
#agdCoordinateList

if reconstructGCode:
    #first, the model before gradient descent is applied is reconstructed
    outputFile = open("output/beforeGradientDescentReconstructed.gcode", "a+")
    outputFile.seek(0)
    outputFile.truncate()
    outputFile.write(';This is a reconstruction of the model encoded in a data strip before the custom gradient descent algorithm is applied\n')
    outputFile.write(';To visualize the contents of this file, open in a 3D GCode viewer or in a slicer such as Cura\n')
    for index, point in enumerate(bgdCoordinateList):
        if point[2] != bgdCoordinateList[index-1][2]:
            outputFile.write('G1 X%f Y%f Z%f\n' % (point[0], point[1], point[2]))
        else:
            if extruderActive[index] == 1:
                outputFile.write('G1 X%f Y%f Z%f E%f\n' % (point[0], point[1], point[2], index))
            else:
                outputFile.write('G1 X%f Y%f Z%f\n' % (point[0], point[1], point[2]))
    outputFile.close()
    
    #now, the model after gradient descent is applied is reconstructed
    outputFile = open("output/afterGradientDescentReconstructed.gcode", "a+")
    outputFile.seek(0)
    outputFile.truncate()
    outputFile.write(';This is a reconstruction of the model encoded in a data strip after the custom gradient descent algorithm is applied\n')
    outputFile.write(';To visualize the contents of this file, open in a 3D GCode viewer or in a slicer such as Cura\n')
    for index, point in enumerate(agdCoordinateList):
        if point[2] != bgdCoordinateList[index-1][2]:
            outputFile.write('G1 X%f Y%f Z%f\n' % (point[0], point[1], point[2]))
        else:
            if extruderActive[index] == 1:
                outputFile.write('G1 X%f Y%f Z%f E%f\n' % (point[0], point[1], point[2], index))
            else:
                outputFile.write('G1 X%f Y%f Z%f\n' % (point[0], point[1], point[2]))
    
print("End Part 10. Completed in " + str(time.perf_counter()-startTime) + "s\n")

#pack the canvas and show the frame
canvas.pack() 
if drawPlot:
    window.mainloop()

print("End of Program\n")
