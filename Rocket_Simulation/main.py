import numpy as np
import pandas as pd
import time as t
import matplotlib.pyplot as plt

'''IMPORTANT DECLARATIONS

North: X+
East: Y+
Up: Z+

Roll: Local Z
Pitch: Local Y
Yaw: Local X

Wind bearing denotes where the wind is coming from, not where the wind is going

AoA is angle of attack

Try to use radians in computation, covert deg inputs to rad
'''

debug = True #True: Output is not performed
print("Writing Output File: " + str(not debug))
rawDatas = [] 
#0       1     2     3         4          5     
#airSim, wind, temp, humitidy, transform, thrust
dataFiles = ["airSim.csv", "wind.csv", "temperature.csv", "humidity.csv", "transform.csv", "thrust.csv"]
sucessImport = True
#create dict of fileNames for later index searching
fileDict = dict()
searchDict = dict()#a dict for showing the current search location in each file
for i in range(0,len(dataFiles)):
    fileDict[dataFiles[i]] = i
    searchDict[dataFiles[i]] = 0

#get input data and return error if can't find a file
datas = [0] * len(dataFiles) #list version of dataframes, big optimization
for i in dataFiles:
    try:
        rawDatas.append(pd.read_csv(i))
        if len(list(rawDatas[fileDict[i]].iloc[:,0])) == 0: 
            sucessImport = False
            print("ERROR: Could not find data from " + i)
        else:
            datas[fileDict[i]] = [0] * rawDatas[fileDict[i]].shape[1]
            for x in range(rawDatas[fileDict[i]].shape[1]):
                datas[fileDict[i]][x] = list(rawDatas[fileDict[i]].iloc[:,x])
               
    except:
        print("ERROR: Could not find file " + i)
        sucessImport = False

#find output files and create new one
fileExists = True
fileIndex = 0
if not debug:
    while fileExists:
        try:
            output = open("output" + str(fileIndex) + ".txt")
            fileIndex+=1
            output.close()
        except:
            fileExists = False
    output = open("output" + str(fileIndex) + ".txt", "w")

#calc AoA and reltive-roll given fwd/up vectors, vel vector and wind vector. Returns two radians
def calcAirAngles(orientation, velocity, windDirection, windSpeed):
    radWind = np.deg2rad(windDirection+180)
    windVector = np.array([np.cos(radWind), np.sin(radWind), 0])*windSpeed
    airVector = windVector+velocity
    if np.linalg.norm(airVector) == 0: #Return zero angles if airspeed is zero, prevent divide by 0
        return [0,0]
    fwd = orientation[0]
    fwdMag = np.linalg.norm(fwd)
    up = orientation[1]

    #AoA math
    AoA = np.arccos(np.dot(fwd,airVector)/(fwdMag*np.linalg.norm(airVector)))

    #Roll math
    project = fwd*(np.dot(fwd,airVector)/(fwdMag**2)) #projection of airVector along fwd vector
    airUp = project-airVector
    roll = 0
    if np.linalg.norm(airUp) != 0:
        roll = np.arccos(np.dot(up,airUp)/(np.linalg.norm(up)*np.linalg.norm(airUp)))

    return [AoA,roll]
#end calcAirAngle

def seekData(fileName, input, outputNum):#given an input, lookup the output in the csv and interpolate
    data = datas[fileDict[fileName]]
    inputs = data[0]
    lastError = inputs[len(inputs)-1] #start with the highest possible error
    result = []
    if input > lastError: 
        result = [0] * outputNum #if the input exceeds the data's scope return zeros
        return result if len(result)>1 else result[0]
    index = searchDict[fileName]
    for i in range(index, len(inputs)):#starts search where it left off, for inputs such as time
        if abs(inputs[i]-input) > abs(lastError):
            break
        else:
            index = i
            searchDict[fileName] = index
            lastError = inputs[i] - input
    #interpolate
    for i in range(0, outputNum):
        if lastError == 0:
            result.append(data[1+i][index])
        else:
            sign = int(np.sign(lastError))
            ratio = lastError/abs(inputs[index-sign]-inputs[index])
            outputs = data[1+i]
            result.append(outputs[index-sign]*ratio + (1-ratio)*outputs[index])
    return result if len(result)>1 else result[0]
#end seekData

def rotate(rotation, eulerChange, dt): #given orientation vectors, peforms euler rotation. Returns two vectors
    newFwd = rotation[0]
    newUp = rotation[1]
    radChange = list(map(lambda x: np.deg2rad(x*dt), eulerChange))
    #perform pitch
    newUp = np.cos(radChange[1])*newUp - np.sin(radChange[1])*newFwd
    newFwd = np.cos(radChange[1])*newFwd + np.sin(radChange[1])*newUp
    #perform roll
    newUp = np.cos(radChange[2])*newUp - np.sin(radChange[2])*np.cross(newFwd,newUp)
    #perform yaw
    newFwd = np.cos(radChange[0])*newFwd - np.sin(radChange[0])*np.cross(newFwd,newUp)
    return [newFwd, newUp]
#end rotate

#supporting function vars
temp = rawDatas[fileDict['airSim.csv']]
airData = []
gridSize = []
for i in range(0,7):
    airData.append(list(temp.iloc[:,i]))
def getCFDData(angleOfAttack, velocity, rollAngle): #given inputs, get cfd sim values
    input = np.array([angleOfAttack, velocity, rollAngle])
    #calculate input's distance with known points
    distances = []
    for i in range(len(airData[0])):
        point = np.array([airData[0][i],airData[1][i],airData[2][i]])
        distance = point-input
        distance = distance[0]**2 + distance[1]**2 + distance[2]**2 #this is actually distance squared
        #distances.append([distance,i])
        
        #faster sort maybe?
        if len(distances) == 0:
            distances.append([distance,i])
        else:
            for x in range(len(distances)-1,-1,-1):
                if distances[x][0] >= distance:
                    if len(distances) == 8:
                        distances.insert(x-1 if x-1>0 else 0,[distance,i])
                        distances.pop(-1)
                    else:
                        distances.insert(x-1 if x-1>0 else 0,[distance,i])
                    break
                else:
                    if len(distances) != 8:
                        distances.insert(x+1,[distance,i])
                    break
    #distances = sorted(distances, key=lambda x: x[0]) #sorts by distances
    #gets closest 8 points
    points = []
    for i in range(0,8):
        index = distances[i][1]
        points.append([airData[0][index],airData[1][index],airData[2][index], index])
    
    #interpolate between all 8 values
    points = sorted(points, key=lambda x: x[2]) #sorts by roll angle

    points[0:4] = sorted(points[0:4], key=lambda x: x[1]) #sorts by vels
    points[4:8] = sorted(points[4:8], key=lambda x: x[1]) #sorts by vels

    points[0:2] = sorted(points[0:2], key=lambda x: x[0]) #sorts by AoA
    points[2:4] = sorted(points[2:4], key=lambda x: x[0]) #sorts by AoA
    points[4:6] = sorted(points[4:6], key=lambda x: x[0]) #sorts by AoA
    points[6:8] = sorted(points[6:8], key=lambda x: x[0]) #sorts by AoA

    #memoizes the grid size, yes that's not a typo
    global gridSize
    if len(gridSize) == 0:
        gridSize = [(points[1][0]-points[0][0]),
                    (points[2][1]-points[0][1]),
                    (points[4][2]-points[0][2])]
    ratios = [(angleOfAttack-points[0][0])/gridSize[0],
              0,
              (velocity-points[0][1])/gridSize[1],
              0,
              (rollAngle-points[0][2])/gridSize[2],
              0]
    ratios[1]=1-ratios[0]
    ratios[3]=1-ratios[2]
    ratios[5]=1-ratios[4]

    valueWeight = [ #weighted values of each point
        ratios[0]*ratios[2],
        ratios[1]*ratios[2],
        ratios[0]*ratios[3],
        ratios[1]*ratios[3],
    ]
    valueWeight = [ #reduce multiply steps
        valueWeight[0]*ratios[4],
        valueWeight[1]*ratios[4],
        valueWeight[2]*ratios[4],
        valueWeight[3]*ratios[4],
        valueWeight[0]*ratios[5],
        valueWeight[1]*ratios[5],
        valueWeight[2]*ratios[5],
        valueWeight[3]*ratios[5],
    ]
    result = [0.0] * 4
    for i in range(3, 7):
        column = airData[i]
        for x in range(0,8):
            result[i-3]+=(valueWeight[x]*column[points[x][3]])
    return result
#end getCFDData
print(getCFDData(0.5,0.5,0.5))#should  output [3.5,0,0,0] with airSimTEST
#main sim loop
if sucessImport:
    #Simulation Variables
    dt = 0.0001 #delta T for simulation
    pos = rawDatas[4]['position'].values
    mass = float(rawDatas[4]['mass'][0])
    tempOrient = rawDatas[4]['orientation'].values #orientation in heading and rail tilt
    vel=np.array([0.0,0.0,0.0])
    time = 0
    rotation = [np.array([0,0,1]), np.array([1,0,0])] #forwardLocal, upwardsLocal
    rotation = rotate(rotation, [0,0,tempOrient[0]], 1)#apply starting roll
    rotation = rotate(rotation, [0,tempOrient[0],0], 1)#apply starting pitch
    rotateRate = np.array([0,0,0])
    #end Simulation Variables

    input("Press Enter to start simulation")
    lastPrint = t.monotonic()
    lastSimTime = 0
    perfTime = 0
    lastPerfTime = 0
    times = []
    heights = []
    while vel[2] >= 0:
        #apply aero forces
        perfTime = t.monotonic()
        windData = seekData("wind.csv", pos[2], 2)
        lastPerfTime+=t.monotonic()-perfTime
        perfTime=t.monotonic()
        airAngles = calcAirAngles(rotation, vel, windData[1], windData[0])
        CFDData = getCFDData(airAngles[0], np.linalg.norm(vel), airAngles[1])


        vel+=(dt*seekData("thrust.csv",time,1)/mass)*rotation[0]-np.array([0,0,9.80665*dt])#gravity and thrust
        pos+=vel*dt
        time+=dt
        times.append(time)
        heights.append(pos[2])
        if t.monotonic()-lastPrint>1:
            #Sim Time Ratio is for measuring program performance
            print(f"Alt: {pos[2]:0.3f}, m/s: {np.linalg.norm(vel):0.3f}, T+: {time:0.3f}, Sim-Time-Ratio: {(time-lastSimTime)/(t.monotonic()-lastPrint):0.3f}, PerfEval {lastPerfTime:0.3f}")
            lastPrint = t.monotonic()
            lastSimTime = time
            lastPerfTime = 0
    plt.plot(np.array(times), np.array(heights))
    plt.show()