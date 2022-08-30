'''
Modified from https://github.com/arimb/PurePursuit

Simulates pathfollowing with the use of only a single path
'''

from cgi import print_form
import configparser
from genericpath import isfile
import math
import numpy as np
import cv2
import imageio
from objects.Robot import Robot
import algorithms.PurePursuit as PurePursuit
import algorithms.Ramsete as Ramsete
import algorithms.SimpleMoveToPoint as SIMPLE
from tqdm import tqdm
import os
import time


config = configparser.ConfigParser()
config.read("config.ini")

robot = Robot(config)


field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])
imageNames = []

scaler = field_length / img_dimension

ctlpts = []
with open(config["CONTROL_POINTS"]["FILE_LOCATION"]) as file:
    for line in file.readlines():
        inner = []
        for i in range(len(line.split(","))):
            if i == 1:
                inner.append(-float(line.split(",")[i])/scaler)
            else:
                inner.append(float(line.split(",")[i])/scaler)
        ctlpts.append(list(inner))

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

width = robot.width
length = robot.length

exportEnabled = int(config["EXPORT"]["ENABLED"])
algorithm = config["CONTROL"]["ALGORITHM"]

pos = robot.pos
robot.angle = math.atan2(path[2][0], path[2][1])
robot.start_pos = path[0]
angle = robot.angle

t = 0
t_i = 0
wheels = [0,0]

dt=0.005

while True:
    
    # call pure pursuit
    if algorithm == "RAMSETE":
        Ramsete.run(robot, "", path)
    elif algorithm == "PURE_PURSUIT":
        PurePursuit.run(robot, "", path)
    else:
        SIMPLE.run(robot, "", ctlpts, False)
        
    key = cv2.waitKey()
    if (key == ord('r')):
        pos = (0,0)
        angle = math.atan2(path[2][0], path[2][1])
        wheels = [0,0]
    else:
        break