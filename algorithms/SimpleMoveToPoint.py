'''
Modified from https://github.com/arimb/PurePursuit
Turn -> Move -> Turn -> Move ...
'''

from cgi import print_form
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

from objects.Robot import Robot

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
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        v[True] -= value

        final_hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return img

def absToLocal(coord):
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
    mindist = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path):
        dist = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if dist < mindist[1]:
            mindist = (i, dist)

    return mindist[0]

def lookahead():        
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

prevAngError = 0
prevctloutput = 0

def distancePID(coord):
    
    e_x = coord[0]
    e_y = coord[1]
    e_theta = math.atan2(coord[0], coord[1])
    
    
    desired_linearVelocity = min(max(0.1*abs(e_y), 10), 1000)
    
    desired_angularVelocity = min(max(0.2*abs(e_theta), 0), 2*math.pi)
            
    
    k = 2 * 0.4 * math.sqrt(desired_angularVelocity * desired_angularVelocity + 1 * desired_linearVelocity * desired_linearVelocity)
    targetLinearVelocity = desired_linearVelocity * math.cos(e_theta) + k * e_y
    
    
    targetAngularVelocity = 0
    if (e_theta != 0):
        targetAngularVelocity = desired_angularVelocity + k * e_theta + (1*desired_linearVelocity*math.sin(e_theta)* e_x) / e_theta
    
    
    leftWheel = targetLinearVelocity + targetAngularVelocity
    rightWheel = targetLinearVelocity - targetAngularVelocity
    
    maxVel = 1000
    
    if leftWheel > maxVel:
        leftWheel = maxVel
    elif leftWheel < -maxVel:
        leftWheel = -maxVel
        
        
    if rightWheel > maxVel:
        rightWheel = maxVel
    elif rightWheel < -maxVel:
        rightWheel = -maxVel
        
    if math.sqrt(e_x**2 + e_y**2) < 2:
        return [0,0]
    return [leftWheel, rightWheel]
    
def anglePID(target_angle):  
    global prevAngError, prevctloutput, angle
    
    
    Kp = 2000
    Kd = 1000
    
    current_angle = 0
    # convert to 0~2pi
    formated_current_angle = formatAngle(angle)
    formated_target_angle = formatAngle(target_angle)
    if formated_target_angle < 0:
        target_angle = 2*math.pi + formated_target_angle
    if formated_current_angle < 0:
        current_angle = 2*math.pi + formated_current_angle   
    else:
        current_angle = formated_current_angle
        
    if (abs(formated_target_angle-formated_current_angle) < abs(target_angle - current_angle)):
        target_angle = formated_target_angle
        current_angle = formated_current_angle
    e_theta = target_angle - current_angle
    
    
    if abs(e_theta) > 0.01 or abs(prevctloutput) > 1: # to ensure the robot rotates to the target angle and settles
        e_theta = target_angle - current_angle
        deriv_theta = e_theta - prevAngError
        
        ctl_output = max(min(e_theta * Kp + deriv_theta * Kd, 1000), -1000)
        prevctloutput = ctl_output
        prevAngError = e_theta
        return [ctl_output, -ctl_output]
    else:
        return [0,0]
        

def draw_path(img):
    try:
        cv2.circle(img, (int(start_pos[0]+path[0][0]), int(start_pos[1]-path[0][1])), 2,
            (255, 0, 255), -1)
        for i in range(2, len(path)):
            cv2.circle(img, (int(start_pos[0]+path[i][0]), int(start_pos[1]-path[i][1])), 2,
                    (255, 0, 255), -1)
            cv2.line(img, (int(start_pos[0]+path[i][0]), int(start_pos[1]-path[i][1])),
                    (int(start_pos[0]+path[i-1][0]), int(start_pos[1]-path[i-1][1])),
                    (255, 0, 255), 1)
    except:
        pass

def draw_robot(img):
    global angle
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
    global pos, angle, wheels
    if event == cv2.EVENT_LBUTTONDOWN:
        pos = [(x-start_pos[0]),(start_pos[1]-y)]
        angle = 0
        wheels = [0, 0]

def run(robot, strat_name, input_path, turnOnly):
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

    if not turnOnly:
        for z in range(1, len(path)-1):
            target = path[z]
            localTarget = absToLocal(target)
            faceAngle = math.atan2(target[0]-pos[0], target[1]-pos[1])
            
            if (z == 1):
                if (abs(target[0]-pos[0]) > 1 or abs(target[1]-pos[1]) > 1):
                    angle = faceAngle
            else:
                # rotate to angle
                while True:
                    last_wheels = wheels
                    wheels = anglePID(faceAngle)
                    if wheels == [0,0]:
                        break
                    
                    for i, w in enumerate(wheels):
                        wheels[i] = last_wheels[i] + min(robot.maxVChange*dt, max(-robot.maxVChange*dt, w-last_wheels[i]))

                    pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
                    angle += math.atan((wheels[0]-wheels[1])/width*dt)
                    
                    draw_robot(img)
                    itt += 1
                
            
            # move to point
            while True:
                last_wheels = wheels
                target = path[z]
                localTarget = absToLocal(target)
                wheels = distancePID(localTarget)
                
                if wheels == [0,0]:
                    break
                
                for i, w in enumerate(wheels):
                    wheels[i] = last_wheels[i] + min(robot.maxVChange*dt, max(-robot.maxVChange*dt, w-last_wheels[i]))

                pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
                angle += math.atan((wheels[0]-wheels[1])/width*dt)
                
                draw_robot(img)
                itt += 1
    
    target = path[len(path)-1]
    localTarget = absToLocal(target)
    faceAngle = math.atan2(target[0]-pos[0], target[1]-pos[1])
    if turnOnly:
        faceAngle = math.atan2(target[0], target[1])
        if abs(target[0]) < 1 and abs(target[1]) < 1:
            faceAngle = angle
            
    # rotate to angle
    while True:
        last_wheels = wheels
        wheels = anglePID(faceAngle)
        
        if wheels == [0,0]:
            break
        
        for i, w in enumerate(wheels):
            wheels[i] = last_wheels[i] + min(robot.maxVChange*dt, max(-robot.maxVChange*dt, w-last_wheels[i]))

        pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
        print(angle)
        angle += math.atan((wheels[0]-wheels[1])/width*dt)
        
        draw_robot(img)
        itt += 1
    
    robot.angle = angle
    if exportEnabled:
        with imageio.get_writer('images/movie.gif', mode='I') as writer:
            print("Writing gif to images/movie.gif")
            for name in tqdm(imageNames):
                writer.append_data(imageio.imread(name))
            print("Done!")