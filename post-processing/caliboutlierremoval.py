import math
import matplotlib.pyplot as plt
from sys import version_info
from collections import deque
from bisect import insort, bisect_left
from itertools import islice
from os import environ

import statistics

if version_info[0] < 3:
    raise "Must be using Python 3"

# def quatertoRPY(x,y,z,w):
#     roll = math.atan2(2.0*(x*w + y*z), 1 - 2*(z*z + w*w))
#     pitch = math.asin(2.0*(x*z - w*y))
#     yaw = math.atan2(2.0*(x*y + z*w), 1 - 2*(y*y + z*z))
#
#     if abs(yaw+pi) < 30*pi/180:
#         yaw = yaw+2*pi
#
#     return([roll,pitch,yaw])


def quatertoRPY(x, y, z, w):
    roll = math.atan2(2.0 * (x * w + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw = math.atan2(2.0 * (x * y + z * w), 1 - 2 * (y * y + z * z))

    if abs(yaw + pi) < 30 * pi / 180:
        yaw = yaw + 2 * pi

    return ([roll, pitch, yaw])


def RPYtoquaternion(yaw, pitch, roll):


    #rotation from inertial frame to the body framee
    three_two_one_rotation = [
        [math.cos(pitch)*math.cos(yaw)  ,  math.cos(pitch)*math.sin(yaw) , -math.sin(pitch) ],
        [math.sin(roll)*math.sin(pitch)*math.cos(yaw)-math.cos(roll)*math.sin(yaw), math.sin(roll)*math.sin(pitch)*math.sin(yaw)+math.cos(roll)*math.cos(yaw) ,  math.sin(roll)*math.cos(pitch)],
        [math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw), math.cos(roll) * math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw),  math.cos(roll) * math.cos(pitch)]
    ]

    #quaternion
    w = 0.5 * math.sqrt(three_two_one_rotation[0][0]+three_two_one_rotation[1][1]+three_two_one_rotation[2][2]+1)
    x = (three_two_one_rotation[1][2]-three_two_one_rotation[2][1])/(4*w)
    y = (three_two_one_rotation[2][0] - three_two_one_rotation[0][2])/(4*w)
    z = (three_two_one_rotation[0][1] - three_two_one_rotation[1][0])/(4*w)

    #other solutions
    # cy = math.cos(yaw * 0.5)
    # sy = math.sin(yaw * 0.5)
    # cr = math.cos(roll * 0.5)
    # sr = math.sin(roll * 0.5)
    # cp = math.cos(pitch * 0.5)
    # sp = math.sin(pitch * 0.5)
    #
    # w = cy * cr * cp + sy * sr * sp
    # x = cy * sr * cp - sy * cr * sp
    # y = cy * cr * sp + sy * sr * cp
    # z = sy * cr * cp - cy * sr * sp


    return ([x,y,z,w])



def running_median(seq, window_size):
    result = []
    if (window_size % 2 == 0):
        oddMovingMedianSize = window_size +1
    else:
        oddMovingMedianSize = window_size
    for i, value in enumerate(seq):
        if i < oddMovingMedianSize / 2:
            medianSample = seq[0: oddMovingMedianSize ]
        elif i > (len(seq) - oddMovingMedianSize / 2):
            medianSample = seq[len(seq) - oddMovingMedianSize:]
        else:
            medianSample = seq[int(i - (oddMovingMedianSize-1) / 2 ):int(i + (oddMovingMedianSize-1) / 2)+1]
        result.append(statistics.median(medianSample))


    return result




pi = math.pi

inputfile = open(environ["HOME"] + '/calib_transforms.txt', 'r')
outputfile = open(environ["HOME"] + '/calib_transforms_no_outliers.m', 'w')

strdata = [line.strip("\n").split(" ") for line in inputfile.readlines()[1:-1]]
data = [[float(value.replace(",", "")) for value in row] for row in strdata]

timestamp = []
x = []
y = []
z = []
roll = []
pitch = []
yaw = []
new_x = x
new_y = y
new_z= z
new_roll = roll
new_pitch = pitch
new_yaw = yaw

#
# x1 = 0
# y1 = 0
# z1 = 0
# w1 = 1
#
# norm = math.sqrt(x1*x1 + y1*y1 + z1*z1+w1*w1)
# print("norm"+str(norm))
#
# print("original " + str(x1)+" "+str(y1)+" "+str(z1)+" "+str(w1))
# euler = quatertoRPY(x1,y1,z1,w1)
# quat = RPYtoquaternion(euler[2],euler[1],euler[0])
# print("results")
# print("solution " +str(quat[0])+" "+str(quat[1])+" "+str(quat[2])+" "+str(quat[3]))

for row in data:
    timestamp.append(row[0])
    x.append(row[1])
    y.append(row[2])
    z.append(row[3])
    try:
    	ro,pit,ya = quatertoRPY(row[4],row[5],row[6],row[7])
    except:
        pass
    roll.append(ro*180/pi)
    pitch.append(pit*180/pi)
    yaw.append(ya*180/pi)

exit = False




plt.interactive('True')
fig = plt.figure()


while(exit != True):
    fig.clear()
    plt.subplot(6, 3, 1)
    plt.plot(range(len(x)), x, "b-")
    plt.ylabel("x")
    plt.xlabel("Measurement Index")

    plt.subplot(6, 3, 4)
    plt.plot(range(len(y)), y, "b-")
    plt.ylabel("y")
    plt.xlabel("Measurement Index")

    plt.subplot(6, 3, 7)
    plt.plot(range(len(z)), z, "b-")
    plt.ylabel("z")
    plt.xlabel("Measurement Index")

    plt.subplot(6, 3, 10)
    plt.plot(range(len(yaw)), yaw, "b-")
    plt.ylabel("yaw")
    plt.xlabel("Measurement Index")
    

    plt.subplot(6, 3, 13)
    plt.plot(range(len(pitch)), pitch, "b-")
    plt.ylabel("pitch")
    plt.xlabel("Measurement Index")


    plt.subplot(6, 3, 16)
    plt.plot(range(len(roll)), roll, "b-")
    plt.ylabel("roll")
    plt.xlabel("Measurement Index")

    # new value
    plt.subplot(6, 3, 3)
    plt.plot(range(len(new_x)), new_x, "b-")
    plt.ylabel("new x")
    plt.xlabel("Measurement Index")

    plt.subplot(6, 3, 6)
    plt.plot(range(len(new_y)), new_y, "b-")
    plt.ylabel("new y")
    plt.xlabel("Measurement Index")

    plt.subplot(6, 3, 9)
    plt.plot(range(len(new_z)), new_z, "b-")
    plt.ylabel("new z")
    plt.xlabel("Measurement Index")

    plt.subplot(6, 3, 12)
    plt.plot(range(len(new_yaw)), new_yaw, "b-")
    plt.ylabel("new yaw")
    plt.xlabel("Measurement Index")

    plt.subplot(6, 3, 15)
    plt.plot(range(len(new_pitch)), new_pitch, "b-")
    plt.ylabel("new pitch")
    plt.xlabel("Measurement Index")


    plt.subplot(6, 3, 18)
    plt.plot(range(len(new_roll)), new_roll, "b-")
    plt.ylabel("new roll")
    plt.xlabel("Measurement Index")


    plt.draw()
    
    userparam = input("Select the parameter you would like to limit (x, y, z, yaw, pitch, roll) or \"exit\" to exit:\n")

    if userparam in ['x', 'y', 'z', 'yaw', 'pitch', 'roll']:
        movingMedianSize = input("Select average median sample of the data in " + userparam + " (or return to cancel):\n")

        if movingMedianSize  == "": movingMedianSize  = None
        else: movingMedianSize  = int(movingMedianSize)

        if movingMedianSize != None:
            if userparam == 'x':
                new_x = running_median(x, int(movingMedianSize))

            elif userparam == 'y':
                new_y = running_median(y,  movingMedianSize)

            elif userparam == 'z':
                new_z = running_median(z,  movingMedianSize)

            elif userparam == 'roll':
                new_roll = running_median(roll,  movingMedianSize)

            elif userparam == 'pitch':
                new_pitch = running_median(pitch,  movingMedianSize)

            elif userparam == 'yaw':
                new_yaw = running_median(yaw, movingMedianSize)


    elif userparam == "exit":
        exit = True
    
    else: print("Please enter a valid parameter.\n")

outputfile.write("tf_cam_to_rgb_optical_calibration_data=[\n")

for index in range(len(new_x)):
    quat = RPYtoquaternion(new_yaw[index], new_pitch[index],new_roll[index])
    outputfile.write( str(timestamp[index])+" "+str(new_x[index])+" "+str(new_y[index])+" "+str(new_z[index])+" "+str(quat[0])+" "+str(quat[1])+" "+str(quat[2])+" "+str(quat[3])+"\n")

outputfile.write("]")

print("Changes written to: " + outputfile.name)

inputfile.close()
outputfile.close()
