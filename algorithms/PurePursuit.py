'''
Modified from https://github.com/arimb/PurePursuit
Pure pursuit path follower
'''

import configparser
from genericpath import isfile
from graphics.StratGrapher import *
import math
import numpy as np
import cv2
import imageio
from tqdm import tqdm
import os
import time

path = []
pos = [0,0]
start_pos = [0,0]
angle = 0
wheels = [0,0]
config = configparser.ConfigParser()
config.read("config.ini")
field_length = 0
img_dimension = 0
imageNames = []

scaler = 0
width = 0
length = 0
curv = 0
look = [0,0]
close = 0
exportEnabled = 0
itt = 0
    
def decrease_brightness(img, value=30):
    # decrease brightness of an image
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v[True] -= value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def absToLocal(coord):
    # absolute to local coordinate
    
    xDist = coord[0] - pos[0]
    yDist = coord[1] - pos[1]
    
    newX = xDist*math.cos(angle) - yDist*math.sin(angle)
    newY = xDist*math.sin(angle) + yDist*math.cos(angle)

    return (newX, newY)

def formatAngle(a):
    # format angle to -π / π
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
    # find the closest point on the path
    
    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path):
        dist = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if dist < mindist[1]:
            mindist = (i, dist)

    return mindist[0]

def lookahead():   
    # find look ahead point
         
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
    # calculate wheel velocity based on curvature and velocity
    
    return  [vel*(1+curv*trackwidth/2), vel*(1-curv*trackwidth/2)]

def draw_path(img):
    # draw path on an image
    
    cv2.circle(img, (int(start_pos[0]+path[0][0]), int(start_pos[1]-path[0][1])), 2,
            (255, 0, 255), -1)
    for i in range(2, len(path)):
        cv2.circle(img, (int(start_pos[0]+path[i][0]), int(start_pos[1]-path[i][1])), 2,
                (255, 0, 255), -1)
        cv2.line(img, (int(start_pos[0]+path[i][0]), int(start_pos[1]-path[i][1])),
                (int(start_pos[0]+path[i-1][0]), int(start_pos[1]-path[i-1][1])),
                (255, 0, 255), 1)


def draw_robot(img, robot, strat_name):  
    # draw robot on an image

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
        if len(strat_name) == 0:
            if itt%6==0:
                cv2.imwrite("images/" + str(itt) + ".png", tmp)
                imageNames.append("images/" + str(itt) + ".png")
        else:
            if robot.itt%6==0:
                cv2.imwrite("images/" + str(robot.itt) + ".png", tmp)
                imageNames.append("images/" + str(robot.itt) + ".png")
    cv2.waitKey(5)

def click(event, x, y, flags, param):
    global pos, angle, wheels
    if event == cv2.EVENT_LBUTTONDOWN:
        pos = [(x-start_pos[0]),(start_pos[1]-y)]
        angle = 0
        wheels = [0, 0]

def run(robot, strat_name, input_path): 
    global config, field_length, img_dimension, scaler, width, length, path, exportEnabled, pos, start_pos, angle, wheels, close, look, curv, imageNames
    field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
    img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])
    imageNames = []

    scaler = robot.scaler

    path = input_path

    width = robot.width
    length = robot.length

    exportEnabled = int(config["EXPORT"]["ENABLED"])

    pos = robot.pos
    angle = robot.angle
    itt = 0
    wheels = [0,0]

    dt=0.005

    field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
    img = decrease_brightness(field, 100)
    start_pos = robot.start_pos 
    del path[0]


    if len(strat_name) != 0:
        graph_paths(img, strat_name)
        graph_ctlpoints(img, strat_name)
    else:
        draw_path(img)
    
    cv2.imshow("img", img)
    cv2.setMouseCallback('img', click)


    while closest() != len(path)-1:
        
        # get lookahead point
        look = lookahead()
        locallook = absToLocal(look)
        close = closest()
            
        # calculate velocity
        curv = (2*locallook[0]) / ((float(config["PATH"]["LOOKAHEAD"])/scaler) **2)
        vel = 0
        if (curv == 0 or abs(1/curv) > 1000):
            vel = robot.maxVel
        else:
            vel = min(abs(robot.maxVel / (curv*180)), robot.maxVel)
        
        last_wheels = wheels
        wheels = turn(curv, vel, width)    

        for i, w in enumerate(wheels):
            wheels[i] = last_wheels[i] + min(robot.maxVChange*dt, max(-robot.maxVChange*dt, w-last_wheels[i]))

        # calculate position and angle
        pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
        angle += math.atan((wheels[0]-wheels[1])/width*dt)

        draw_robot(img, robot, strat_name)
        if len(strat_name) == 0:
            itt += 1
        else:
            robot.itt += 1
        
    robot.angle = angle
    robot.imageNames += imageNames
    if exportEnabled and len(strat_name) == 0:
        with imageio.get_writer('images/movie.gif', mode='I') as writer:
            print("Writing gif to images/movie.gif")
            for name in tqdm(imageNames):
                writer.append_data(imageio.imread(name))
            print("Done!")