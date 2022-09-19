import cv2
import configparser
import math
import numpy as np
import sys
import os

def graph_ctlpoints(img, strat_name):
    # READ CONFIG FILE
    config = configparser.ConfigParser()
    config.read("config.ini")

    field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
    img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])

    scaler = field_length / img_dimension
    
    strat_dir = f"strats/{strat_name}/"
    actionFile = open(strat_dir + "actions.csv", "r")
    actions = []
    for line in actionFile.readlines():
        inner = []
        for l in line.split(","):
            if l.strip().isdigit():
                inner.append(int(l))
            else:
                inner.append(l)
        actions.append(inner)
    
    print(actions)
    for k, file in enumerate(os.listdir(f"strats/{strat_name}/control_points/")):
        path = []
        start = [0,0]
        if (actions[int(file.split(".")[0])][0] == "SIMPLE"):
            with open(f"strats/{strat_name}/control_points/" + file) as f:
                for line in f.readlines():
                    inner = []
                    for i in range(len(line.split(","))):
                        if i == 1:
                            inner.append(-float(line.split(",")[i])/scaler)
                        else:
                            inner.append(float(line.split(",")[i])/scaler)
                    path.append(list(inner))
            start = path[0]
            del path[0]
            for i in range(1, len(path)):
                color = (0, 200, 200)
                if (i == len(path)-1): # facing control point should be darkened
                    color = (0, 100, 100)
                
                cv2.circle(img, (int(start[0]+path[i][0]),
                                int(start[1]-path[i][1])),
                        3, color, -1)
                cv2.line(img, (int(start[0]+path[i][0]),
                                int(start[1]-path[i][1])),
                        (int(start[0] + path[i-1][0]),
                        int(start[1] - path[i-1][1])),
                        color, 2)
    
def graph_paths(img, strat_name):
    # READ CONFIG FILE
    config = configparser.ConfigParser()
    config.read("config.ini")

    field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
    img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])

    scaler = field_length / img_dimension
    
    strat_dir = f"strats/{strat_name}/"
    actionFile = open(strat_dir + "actions.csv", "r")
    actions = []
    for line in actionFile.readlines():
        inner = []
        for l in line.split(","):
            if l.strip().isdigit():
                inner.append(int(l))
            else:
                inner.append(l)
        actions.append(inner)
    
    
    for k, file in enumerate(os.listdir(f"strats/{strat_name}/paths/")):
        try:
            path = []
            start = [0,0]
            if (actions[int(file.split(".")[0])][0] != "SIMPLE"):
                with open(f"strats/{strat_name}/paths/" + file) as f:
                    for line in f.readlines():
                        inner = []
                        for i in range(len(line.split(","))):
                            if i == 1:
                                inner.append(-float(line.split(",")[i])/scaler)
                            else:
                                inner.append(float(line.split(",")[i])/scaler)
                        path.append(list(inner))
                start = path[0]
                del path[0]
                
                color = (200, 200, 200)
                if actions[int(file.split(".")[0])][0] == "PURE_PURSUIT":
                    color = (255, 0, 255)
                elif actions[int(file.split(".")[0])][0] == "RAMSETE":
                    color = (255, 200, 100)
                for i in range(1, len(path)):
                    cv2.circle(img, (int(start[0]+path[i][0]),
                                    int(start[1]-path[i][1])),
                            2, color, -1)
                    cv2.line(img, (int(start[0]+path[i][0]),
                                    int(start[1]-path[i][1])),
                            (int(start[0] + path[i-1][0]),
                            int(start[1] - path[i-1][1])),
                            color, 1)
        except:
            pass
    