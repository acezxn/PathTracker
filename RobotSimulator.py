'''
Modified from https://github.com/arimb/PurePursuit
'''

from cgi import print_form
import configparser
from genericpath import isfile
import math
import numpy as np
import cv2
import imageio
from tqdm import tqdm
import os
import time

def decrease_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v[True] -= value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def absToLocal(coord):
    global pos, angle
    xDist = coord[0] - pos[0];
    yDist = coord[1] - pos[1];
    
    newX = xDist*math.cos(angle) - yDist*math.sin(angle);
    newY = xDist*math.sin(angle) + yDist*math.cos(angle);

    return (newX, newY)

def formatAngle(a):
    sign = 1
    if a < 0:
        sign = -1
    positive_a = abs(a)
    mod = positive_a % (2*math.pi)
    if (mod < math.pi):
        return sign*mod
    else:
        return sign*(mod-2*math.pi)
    
def closest():
    global path, pos

    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path):
        dist = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if dist < mindist[1]:
            mindist = (i, dist)

    return mindist[0]
def lookahead():
    global path, t, t_i, pos
    
    radius = float(config["PATH"]["LOOKAHEAD"])/scaler
    for i in range(2, len(path)):
        coord = path[i]
        prevCoord = path[i-1]
        coordDist = math.sqrt((coord[0]-pos[0])**2 + (coord[1]-pos[1])**2)
        prevCoordDist = math.sqrt((prevCoord[0]-pos[0])**2 + (prevCoord[1]-pos[1])**2)
        if (coordDist > (radius) and prevCoordDist < (radius) and closest() < i):
            prevX = prevCoord[0]
            prevY = prevCoord[1]
            currX = coord[0]
            currY = coord[1]
            minT = 0
            maxT = 1
            newX = prevX
            newY = prevY

            iterations = 10
            for z in range(iterations):
                midT = (minT + maxT) / 2.0
                newX = prevX * (1 - midT) + currX * midT
                newY = prevY * (1 - midT) + currY * midT
                
                look = (newX, newY)
                lookDist = math.sqrt((newX-pos[0])**2 + (newY-pos[1])**2)
                if (lookDist < radius - 0.05):
                    minT = midT + 0.05
                elif (lookDist > radius + 0.05):
                    maxT = midT - 0.05
                else:
                    return look
    endDist = math.sqrt((path[len(path)-1][0]-pos[0])**2 + (path[len(path)-1][1]-pos[1])**2)
    if (endDist < radius):
        return path[len(path)-1]
    return path[closest()]

def turn(curv, vel, trackwidth):
    return  [vel*(1+curv*trackwidth/2), vel*(1-curv*trackwidth/2)]

def draw_path(img):
    global path, start_pos
    cv2.circle(img, (int(start_pos[0]+path[0][0]), int(start_pos[1]-path[0][1])), 2,
               (255, 0, 255), -1)
    for i in range(2, len(path)):
        cv2.circle(img, (int(start_pos[0]+path[i][0]), int(start_pos[1]-path[i][1])), 2,
                  (255, 0, 255), -1)
        cv2.line(img, (int(start_pos[0]+path[i][0]), int(start_pos[1]-path[i][1])),
                 (int(start_pos[0]+path[i-1][0]), int(start_pos[1]-path[i-1][1])),
                 (255, 0, 255), 1)
def draw_robot(img):
    tmp = img.copy()
    cv2.circle(tmp, (int(start_pos[0]+pos[0]), int(start_pos[1]-pos[1])), 4,
               (0, 255, 255), -1)
    cv2.drawContours(tmp, [np.array([(start_pos[0] + (pos[0]+length/2*math.sin(angle)-width/2*math.cos(angle)),
                                      start_pos[1] + (pos[1]+length/2*math.cos(angle)+width/2*math.sin(angle))*-1),
                                     (start_pos[0] + (pos[0]+length/2*math.sin(angle)+width/2*math.cos(angle)),
                                      start_pos[1] + (pos[1]+length/2*math.cos(angle)-width/2*math.sin(angle))*-1),
                                     (start_pos[0] + (pos[0]-length/2*math.sin(angle)+width/2*math.cos(angle)),
                                      start_pos[1] + (pos[1]-length/2*math.cos(angle)-width/2*math.sin(angle))*-1),
                                     (start_pos[0] + (pos[0]-length/2*math.sin(angle)-width/2*math.cos(angle)),
                                      start_pos[1] + (pos[1]-length/2*math.cos(angle)+width/2*math.sin(angle))*-1)])
                     .reshape((-1,1,2)).astype(np.int32)], 0, (0, 255, 255), 2)
    cv2.circle(tmp, (int(start_pos[0]+pos[0]), int(start_pos[1]-pos[1])), int(int(config["PATH"]["LOOKAHEAD"])/scaler), (0, 255, 0), 1)
    cv2.circle(tmp, (int(start_pos[0]+path[close][0]), int(start_pos[1]-path[close][1])), 4,
               (255, 0, 255), -1)
    cv2.circle(tmp, (int(start_pos[0]+look[0]), int(start_pos[1]-look[1])), 4, (0,255,0), -1)

    try:
        x3 = (pos[0]+look[0])/2
        y3 = -(pos[1]+look[1])/2
        q = math.sqrt((pos[0]-look[0])**2 + (pos[1]-look[1])**2)
        x = x3 - math.sqrt(1/curv**2 - (q/2)**2) * (pos[1]-look[1])/q * np.sign(curv)
        y = y3 - math.sqrt(1/curv**2 - (q/2)**2) * (pos[0]-look[0])/q * np.sign(curv)
        cv2.circle(tmp, (int(x+start_pos[0]), int(y+start_pos[1])), int(abs(1/curv)), (0,255,255), 1)
    except:
        pass

    cv2.line(tmp,
             (int(start_pos[0] + (pos[0]+length/2*math.sin(angle)-width/2*math.cos(angle))),
              int(start_pos[1] + (pos[1]+length/2*math.cos(angle)+width/2*math.sin(angle))*-1)),
             (int(start_pos[0] + (pos[0]+(length/2+wheels[0]/5)*math.sin(angle)-width/2*math.cos(angle))),
              int(start_pos[1] + (pos[1]+(length/2+wheels[0]/5)*math.cos(angle)+width/2*math.sin(angle))*-1)),
             (0,0,255), 2)
    cv2.line(tmp,
             (int(start_pos[0] + (pos[0]+length/2*math.sin(angle)+width/2*math.cos(angle))),
              int(start_pos[1] + (pos[1]+length/2*math.cos(angle)-width/2*math.sin(angle))*-1)),
             (int(start_pos[0] + (pos[0]+(length/2+wheels[1]/5)*math.sin(angle)+width/2*math.cos(angle))),
              int(start_pos[1] + (pos[1]+(length/2+wheels[1]/5)*math.cos(angle)-width/2*math.sin(angle))*-1)),
             (0,0,255), 2)

    cv2.imshow("img", tmp)
    if exportEnabled:
        if itt%6==0:
            cv2.imwrite("images/" + str(itt) + ".png", tmp)
            imageNames.append("images/" + str(itt) + ".png")
    cv2.waitKey(5)

def click(event, x, y, flags, param):
    global pos, angle, t, t_i, wheels
    if event == cv2.EVENT_LBUTTONDOWN:
        pos = ((x-start_pos[0]),(start_pos[1]-y))
        angle = 0
        t = 0
        t_i = 0
        wheels = [0, 0]


config = configparser.ConfigParser()
config.read("config.ini")

field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])
imageNames = []

scaler = field_length / img_dimension

path = []
with open(config["PATH"]["FILE_LOCATION"]) as file:
    for line in file.readlines():
        inner = []
        for i in range(len(line.split(","))):
            if i == 1:
                inner.append(-float(line.split(",")[i])/scaler)
            else:
                inner.append(float(line.split(",")[i])/scaler)
        path.append(list(inner))

width = float(config["ROBOT"]["TRACKWIDTH"]) / scaler
length = float(config["ROBOT"]["LENGTH"]) / scaler

exportEnabled = int(config["EXPORT"]["ENABLED"])
algorithm = config["CONTROL"]["ALGORITHM"]

pos = (0,0)
angle = math.atan2(path[2][0], path[2][1])
t = 0
t_i = 0
wheels = [0,0]

dt=0.005

field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
img = decrease_brightness(field, 100)
start_pos = path[0]
del path[0]


draw_path(img)
cv2.imshow("img", img)
cv2.setMouseCallback('img', click)

itt = 0
while True:
    while closest() != len(path)-1:
        
        look = lookahead()
        locallook = absToLocal(look)
        close = closest()
            
        if algorithm == "RAMSETE":
            e_x = locallook[0]
            e_y = locallook[1]
            e_theta = math.atan2(locallook[0], locallook[1])
            
            
            desired_linearVelocity = min(max(0.04*abs(e_y), 10), 500)
            
            desired_angularVelocity = min(max(0.1*abs(e_theta), 0), 2*math.pi)
                    
            
            k = 2 * 0.4 * math.sqrt(desired_angularVelocity * desired_angularVelocity + 0.5 * desired_linearVelocity * desired_linearVelocity)
            targetLinearVelocity = desired_linearVelocity * math.cos(e_theta) + k * e_y
            
            
            targetAngularVelocity = 0
            if (e_theta != 0):
                targetAngularVelocity = desired_angularVelocity + k * e_theta + (0.5*desired_linearVelocity*math.sin(e_theta)* e_x) / e_theta
            
            last_wheels = wheels
            
            leftWeel = targetLinearVelocity + targetAngularVelocity
            rightWheel = targetLinearVelocity - targetAngularVelocity
            if leftWeel > float(config["VELOCITY"]["MAX_VEL"]):
                leftWeel = float(config["VELOCITY"]["MAX_VEL"])
            elif leftWeel < -float(config["VELOCITY"]["MAX_VEL"]):
                leftWeel = -float(config["VELOCITY"]["MAX_VEL"])
                
            if rightWheel > float(config["VELOCITY"]["MAX_VEL"]):
                rightWheel = float(config["VELOCITY"]["MAX_VEL"])
            elif rightWheel < -float(config["VELOCITY"]["MAX_VEL"]):
                rightWheel = -float(config["VELOCITY"]["MAX_VEL"])
                
            
            wheels = [leftWeel, rightWheel]
            
            for i, w in enumerate(wheels):
                wheels[i] = last_wheels[i] + min(float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, max(-float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, w-last_wheels[i]))

            pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
            angle += math.atan((wheels[0]-wheels[1])/width*dt)
            # print(str(wheels) + ", " + str(angle))

            draw_robot(img)
        elif algorithm == "PURE_PURSUIT":
            curv = (2*locallook[0]) / ((float(config["PATH"]["LOOKAHEAD"])/scaler) **2)
            vel = 0
            if (curv == 0 or abs(1/curv) > 1000):
                vel = 600
            else:
                vel = min(abs(600 / (curv*180)), 600)
            
            last_wheels = wheels
            wheels = turn(curv, vel, width)    

            for i, w in enumerate(wheels):
                wheels[i] = last_wheels[i] + min(float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, max(-float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, w-last_wheels[i]))

            pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
            angle += math.atan((wheels[0]-wheels[1])/width*dt)
            # print(str(wheels) + ", " + str(angle))

            st = time.time()
            draw_robot(img)
            et = time.time()
            # print(et-st)
        itt += 1
        
    if exportEnabled:
        with imageio.get_writer('images/movie.gif', mode='I') as writer:
            print("Writing gif to images/movie.gif")
            for name in tqdm(imageNames):
                writer.append_data(imageio.imread(name))
            print("Done!")
        
    key = cv2.waitKey()
    if (key == ord('r')):
        pos = (0,0)
        angle = math.atan2(path[2][0], path[2][1])
        wheels = [0,0]
    else:
        break