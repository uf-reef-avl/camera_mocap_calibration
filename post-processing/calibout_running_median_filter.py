import math
import matplotlib.pyplot as plt
from sys import version_info
from collections import deque
from bisect import insort, bisect_left
from itertools import islice
from os import environ
import numpy as np

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


output_nwu_path = environ["HOME"] + '/extrinsec_parameter_nwu.yaml'
output_ned_path = environ["HOME"] + '/extrinsec_parameter_ned.yaml'
inputfile = open(environ["HOME"] + '/calib_transforms.txt', 'r')
outputfile_NWU = open(output_nwu_path, 'w')
outputfile_NED = open(output_ned_path, 'w')


strdata = [line.strip("\n").split(" ") for line in inputfile.readlines()[1:-1]]
print(strdata)
data = [[float(value.replace(",", "")) for value in row] for row in strdata]

time_stamp = []
x = []
y = []
z = []
qx = []
qy = []
qz = []
qw = []
roll = []
pitch = []
yaw = []
new_time_stamp = time_stamp
new_x = x
new_y = y
new_z= z
new_roll = roll
new_pitch = pitch
new_yaw = yaw
new_qx = qx 
new_qy= qy 
new_qz = qz
new_qw = qw 


for row in data:
    time_stamp.append(row[0])
    x.append(row[1])
    y.append(row[2])
    z.append(row[3])
    qx.append(row[4])
    qy.append(row[5])
    qz.append(row[6])
    qw.append(row[7])    
	

    try:
    	ro,pit,ya = quatertoRPY(row[4],row[5],row[6],row[7])
    except:
        pass
    roll.append(ro*180/pi)
    pitch.append(pit*180/pi)
    yaw.append(ya*180/pi)

exit = False
save = False



plt.interactive('True')


while(exit != True):
    fig = plt.figure(1)
    fig.clear()

    plt.subplot(7, 2, 1)
    plt.plot(range(len(x)), x, "b-")
    plt.ylabel("x")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 3)
    plt.plot(range(len(y)), y, "b-")
    plt.ylabel("y")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 5)
    plt.plot(range(len(z)), z, "b-")
    plt.ylabel("z")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 7)
    plt.plot(range(len(qx)), qx, "b-")
    plt.ylabel("qx")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 9)
    plt.plot(range(len(qy)), qy, "b-")
    plt.ylabel("qy")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 11)
    plt.plot(range(len(qz)), qz, "b-")
    plt.ylabel("qz")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 13)
    plt.plot(range(len(qw)), qw, "b-")
    plt.ylabel("qw")
    plt.xlabel("Measurement Index")


    plt.subplot(7, 2, 6)
    plt.plot(range(len(yaw)), yaw, "b-")
    plt.ylabel("yaw")
    plt.xlabel("Measurement Index")
    

    plt.subplot(7, 2, 8)
    plt.plot(range(len(pitch)), pitch, "b-")
    plt.ylabel("pitch")
    plt.xlabel("Measurement Index")


    plt.subplot(7, 2, 10)
    plt.plot(range(len(roll)), roll, "b-")
    plt.ylabel("roll")
    plt.xlabel("Measurement Index")





    # new value
    fig = plt.figure(2)
    fig.clear()

    plt.subplot(7, 2, 1)
    plt.plot(range(len(new_x)), new_x, "b-")
    plt.ylabel("new x")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 3)
    plt.plot(range(len(new_y)), new_y, "b-")
    plt.ylabel("new y")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 5)
    plt.plot(range(len(new_z)), new_z, "b-")
    plt.ylabel("new z")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 7)
    plt.plot(range(len(new_qx)), new_qx, "b-")
    plt.ylabel("new qx")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 9)
    plt.plot(range(len(new_qy)), new_qy, "b-")
    plt.ylabel("new_qy")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 11)
    plt.plot(range(len(new_qz)), new_qz, "b-")
    plt.ylabel("new_qz")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 13)
    plt.plot(range(len(new_qw)), new_qw, "b-")
    plt.ylabel("new_qw")
    plt.xlabel("Measurement Index")


    plt.subplot(7, 2, 6)
    plt.plot(range(len(new_yaw)), new_yaw, "b-")
    plt.ylabel("new yaw")
    plt.xlabel("Measurement Index")

    plt.subplot(7, 2, 8)
    plt.plot(range(len(new_pitch)), new_pitch, "b-")
    plt.ylabel("new pitch")
    plt.xlabel("Measurement Index")


    plt.subplot(7, 2, 10)
    plt.plot(range(len(new_roll)), new_roll, "b-")
    plt.ylabel("new roll")
    plt.xlabel("Measurement Index")




    plt.draw()
    
    userparam = input("Select the parameter you would like to limit (x, y, z, qx, qy, qz, qw, position, orientation, all) or \"save\" to save or \"exit\" to exit:\n")

    if userparam in ['x', 'y', 'z','qx','qy','qz','qw', 'position','orientation', 'all']:
        movingMedianSize = input("Select the window size of the median filter for " + userparam + " (or return to cancel):\n")

        if movingMedianSize  == "": movingMedianSize  = None
        else: movingMedianSize  = int(movingMedianSize)

        if movingMedianSize != None:
            if userparam == 'x':
                new_x = running_median(x, int(movingMedianSize))

            elif userparam == 'y':
                new_y = running_median(y,  int(movingMedianSize))

            elif userparam == 'z':
                new_z = running_median(z,  int(movingMedianSize))

            elif userparam == 'qx':
                new_qx = running_median(qx, int(movingMedianSize))

            elif userparam == 'qy':
                new_qy = running_median(qy,  int(movingMedianSize))

            elif userparam == 'qz':
                new_qz = running_median(qz,  int(movingMedianSize))

            elif userparam == 'qw':
                new_qw = running_median(qw,  int(movingMedianSize))

            elif userparam == 'orientation':
                new_qx = running_median(qx, int(movingMedianSize))
                new_qy = running_median(qy,  int(movingMedianSize))
                new_qz = running_median(qz,  int(movingMedianSize))
                new_qw = running_median(qw,  int(movingMedianSize))
                new_roll = []
                new_pitch = []
                new_yaw = []		
                for i in range(len(new_qx)):
                    temp_roll,temp_pitch,temp_yaw = quatertoRPY(new_qx[i],new_qy[i],new_qz[i],new_qw[i])
                    new_roll.append(temp_roll)
                    new_pitch.append(temp_pitch)
                    new_yaw.append(temp_yaw)

            elif userparam == 'position':
                new_x = running_median(x, int(movingMedianSize))
                new_y = running_median(y,  int(movingMedianSize))
                new_z = running_median(z,  int(movingMedianSize))


            elif userparam == 'all':
                new_x = running_median(x, int(movingMedianSize))
                new_y = running_median(y, int(movingMedianSize))
                new_z = running_median(z, int(movingMedianSize))
                new_qx = running_median(qx, int(movingMedianSize))
                new_qy = running_median(qy,  int(movingMedianSize))
                new_qz = running_median(qz,  int(movingMedianSize))
                new_qw = running_median(qw,  int(movingMedianSize))
                new_roll = []
                new_pitch = []
                new_yaw = []		
                for i in range(len(new_qx)):
                    temp_roll,temp_pitch,temp_yaw = quatertoRPY(new_qx[i],new_qy[i],new_qz[i],new_qw[i])
                    new_roll.append(temp_roll)
                    new_pitch.append(temp_pitch)
                    new_yaw.append(temp_yaw)

    elif userparam == "save":
        exit = True
        save= True

    elif userparam == "exit":
        exit = True
    
    else: print("Please enter a valid parameter.\n")

if save == True:
    x_average = np.average(np.array([new_x]))
    y_average = np.average(np.array([new_y]))
    z_average = np.average(np.array([new_z]))
    qx_average = np.average(np.array([new_qx]))
    qy_average = np.average(np.array([new_qy]))
    qz_average = np.average(np.array([new_qz]))
    qw_average = np.average(np.array([new_qw]))
    norm = np.linalg.norm(np.array([qx_average,qy_average,qz_average,qw_average]))
    qx_average = qx_average / norm
    qy_average = qy_average / norm
    qz_average = qz_average / norm
    qw_average = qw_average / norm
    outputfile_NWU.write("body_to_camera: { \n qx: "+ str(qx_average) +",\n qy: "+ str(qy_average) +",\n qz: "+ str(qz_average) +",\n qw: "+ str(qw_average) +",\n x: "+ str(x_average) +",\n y: "+ str(y_average) +",\n z: "+ str(z_average) +",\n }")
    outputfile_NED.write("body_to_camera: { \n qx: " + str(qx_average) + ",\n qy: " + str(-qy_average) + ",\n qz: " + str(-qz_average) + ",\n qw: "+ str(qw_average) +",\n x: " + str(x_average) + ",\n y: " + str(-y_average) + ",\n z: " + str(-z_average) + ",\n }")
    print("The result have been exported to the files: "+output_nwu_path+" and "+output_nwu_path)

outputfile_NWU.close()
outputfile_NED.close()
inputfile.close()
