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

Try to use radians in computation, covert deg inputs to rad
'''

debug = True #True: Output is not performed
print("Debug Mode: " + str(debug))
datas = [] 
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
for i in dataFiles:
    try:
        datas.append(pd.read_csv(i))
        if len(list(datas[fileDict[i]].iloc[:,0])) == 0: 
            sucessImport = False
            print("ERROR: Could not find data from " + i)
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

#calculate angle of attack given orientation vector, velocity vector and wind vector
def calcAoA(orientation, velocity, windDirection, windSpeed):
    radWind = np.deg2rad(windDirection+180)
    windVector = np.array([np.cos(radWind), np.sin(radWind), 0])*windSpeed
    airVector = windVector+velocity
    angle = np.dot(orientation,airVector)/(np.linalg.norm(orientation)*np.linalg.norm(airVector))
    return angle
#end calcAoa

def seekData(fileName, input, outputNum=1):#given an input, lookup the output in the csv and interpolate
    data = datas[fileDict[fileName]]
    inputs = list(data.iloc[:,0])
    lastError = inputs[len(inputs)-1] #start with the highest possible error
    if input > lastError: return 0 #if the input exceeds the data's scope, return 0
    index = searchDict[fileName]
    for i in range(index, len(inputs)):#starts search where it left off, for inputs such as time
        if abs(inputs[i]-input) > abs(lastError):
            break
        else:
            index = i
            searchDict[fileName] = index
            lastError = inputs[i] - input
    #interpolate
    result = []
    for i in range(0, outputNum):
        if lastError == 0:
            result.append(list(data.iloc[:,1+i])[index])
        else:
            sign = int(np.sign(lastError))
            ratio = lastError/abs(inputs[index-sign]-inputs[index])
            outputs = list(data.iloc[:,1+i])
            result.append(outputs[index-sign]*ratio + (1-ratio)*outputs[index])
    return result if len(result)>1 else result[0]
#end seekData

def rotate(rotation, eulerChange, dt):
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

def calcAero(angleOfAttack, velocity, rollAngle, outputLen): #WIP
    data = datas[fileDict['airSim.csv']]
    AoAs = list(data.iloc[:,0])
    vels = list(data.iloc[:,1])
    rolls = list(data.iloc[:,2])
    input = np.array([angleOfAttack, velocity, rollAngle])

    #calculate input's distance with known points
    distances = []
    for i in range(len(AoAs)):
        point = np.array([AoAs[i],vels[i],rolls[i]])
        distance = np.linalg.norm(point-input)
        distances.append([distance,i])
    distances = sorted(distances, key=lambda x: x[0]) #sorts by distances

    #gets closest 8 points
    points = []
    for i in range(0,8):
        index = distances[i][1]
        points.append([AoAs[index],vels[index],rolls[index], index])
    
    #interpolate between all 8 values
    points = sorted(points, key=lambda x: x[2]) #sorts by roll angle

    points[0:4] = sorted(points[0:4], key=lambda x: x[1]) #sorts by vels
    points[4:8] = sorted(points[4:8], key=lambda x: x[1]) #sorts by vels

    points[0:2] = sorted(points[0:2], key=lambda x: x[0]) #sorts by AoA
    points[2:4] = sorted(points[2:4], key=lambda x: x[0]) #sorts by AoA
    points[4:6] = sorted(points[4:6], key=lambda x: x[0]) #sorts by AoA
    points[6:8] = sorted(points[6:8], key=lambda x: x[0]) #sorts by AoA

    print(points[1][0]-points[0][0])
    print(points[2][1]-points[1][1])
    print(points[4][2]-points[3][2])
    ratios = [angleOfAttack

    ]

    return distances
#end calcAero

#calcAero(0.5,0.5,0.5)

if sucessImport:
    #Simulation Variables
    dt = 0.0001 #delta T for simulation
    pos = datas[4]['position'].values
    mass = float(datas[4]['mass'][0])
    tempOrient = datas[4]['orientation'].values #orientation in heading and rail tilt
    vel=np.array([0.0,0.0,0.0])
    time = 0
    rotation = [np.array([0,0,1]), np.array([1,0,0])] #forwardLocal, upwardsLocal
    rotation = rotate(rotation, [0,0,tempOrient[0]], 1)#apply starting roll
    rotation = rotate(rotation, [0,tempOrient[0],0], 1)#apply starting pitch
    #end Simulation Variables

    input("Press Enter to start simulation")
    lastPrint = t.monotonic()
    times = []
    heights = []
    while vel[2] >= 0:
        vel+=(dt*seekData("thrust.csv",time)/mass)*rotation[0]-np.array([0,0,9.8*dt])#gravity and thrust
        pos+=vel*dt
        #print(pos)
        time+=dt
        times.append(time)
        heights.append(pos[2])
        if t.monotonic()-lastPrint>1:
            print(f"{pos[2]:0.3f}, {np.linalg.norm(vel):0.3f}, {time:0.3f}")
            lastPrint = t.monotonic()
    plt.plot(np.array(times), np.array(heights))
    plt.show()
    #solve for vector orientation