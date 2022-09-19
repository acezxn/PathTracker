'''
Modified from https://github.com/arimb/PurePursuit

Generates paths for single simulation and strategies
'''

import cv2
import configparser
import math
import numpy as np
import sys
import os
from graphics.StratGrapher import *

def decrease_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    v[True] -= value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

# HANDLE MOUSE EVENTS FOR SELECTION
def click(event, x, y, flags, param):
    global waypoints, img, start_pos, mouse_down
    try:
        if mouse_down:
            tmp = img.copy()
            place_point(tmp, x, y, flags&cv2.EVENT_FLAG_SHIFTKEY, False)
            cv2.imshow("Field", tmp)
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_down = True
        if event == cv2.EVENT_LBUTTONUP:
            mouse_down = False
            if len(waypoints) == 0:
                start_pos = (x,y)
            place_point(img, x, y, flags&cv2.EVENT_FLAG_SHIFTKEY, True)
            cv2.imshow("Field", img)
    except Exception as e:
        print(e)

def place_point(img, x, y, shift, add):
    global waypoints
    if shift and len(waypoints)>0:
        if abs(x-(start_pos[0]+waypoints[-1][0])) < abs(y-(start_pos[1]-waypoints[-1][1])):
            x = start_pos[0] + waypoints[-1][0]
        else:
            y = start_pos[1] - waypoints[-1][1]
            
    cv2.circle(img, (x, y), 3, (255, 255, 255), -1)
    if len(waypoints)>0:
        cv2.line(img, (x, y), (start_pos[0]+waypoints[-1][0], start_pos[1]-waypoints[-1][1]), (255, 255, 255), 2)
    if add:
        waypoints.append((x - start_pos[0], start_pos[1] - y))
        print("WP", (x - start_pos[0], start_pos[1] - y))
        control_points.append((x - start_pos[0], start_pos[1] - y)) 
    
pathNo = 0
isdeleting = False

def generate_path(initial_position, strat_name):
    global waypoints, control_points, start_pos, mouse_down, img, pathNo, isdeleting
    while True:
        
        # INITIALIZE VALUES
        waypoints = []
        control_points = []
        mouse_down = False
        
        # READ CONFIG FILE
        config = configparser.ConfigParser()
        config.read("config.ini")
        
        field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
        img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])

        scaler = field_length / img_dimension # scaler for cm conversion
        scalerPCT = 100 / img_dimension # scaler for percentage conversion
        
        if isdeleting:
            print(pathNo)
            if pathNo >= 1:
                prevStartPos = open(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + f"/control_points/{pathNo-1}.csv", "r").read().split("\n")[0].split(",")
                coord = open(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + f"/control_points/{pathNo-1}.csv", "r").read().split("\n")[-3].split(",")
                start_pos = (int((float(prevStartPos[0]) + float(coord[0]))/ scaler), -int((float(prevStartPos[1]) - float(coord[1])) / scaler))
                print(start_pos)
                waypoints.append((0, 0))
                waypoints.append((0, 0))
                control_points.append((0, 0))
                control_points.append((0, 0))
            else:
                start_pos = (0,0)
        else:
            if len(initial_position) == 0:
                start_pos = (0,0)
            else:
                start_pos = initial_position
                waypoints.append((0, 0))
                waypoints.append((0, 0))
                control_points.append((0, 0))
                control_points.append((0, 0))



        # READ & SHOW IMAGE, SET OPENCV PROPERTIES
        img = decrease_brightness(cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"]), 100)
        
        if len(initial_position) != 0:          
            graph_ctlpoints(img, strat_name)
            graph_paths(img, strat_name)
        
        cv2.imshow("Field", img)
        
            
        cv2.setMouseCallback("Field", click)
        
        
            
        key = cv2.waitKey()
        
        if key == 27: # esc key
            return []
        elif key == 127: # backspace key
            # delete previous path
            data = ""
            with open(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + "/actions.csv", "r") as file:
                data = file.read()
            dataList = data.split("\n")
            print("PG", dataList)
            idx = int(dataList[-2].split(",")[1])
            del dataList[-2]
            data = "\n".join(dataList)
            with open(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + "/actions.csv", "w") as file:
                file.write(data)
                
            
            
            os.remove(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + f"/control_points/{idx}.csv")
            os.remove(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + f"/coordinates/{idx}.csv")
            os.remove(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + f"/paths/{idx}.csv")   
            pathNo -= 1
            isdeleting = True
            continue
            
        elif key != 13: # not enter key
            continue
        
        total_waypoints = []
        # MAKE SURE AT LEAST 4 POINTS SELECTED
        print(len(waypoints), start_pos)
        if len(waypoints) >= 4:
            
        
            # Creates cutmull rom spline curve
            total_waypoints.append(start_pos)
            for t in np.arange(0, len(waypoints)-3, 0.05):
                p1 = waypoints[int(t)]
                p2 = waypoints[int(t)+1]
                p3 = waypoints[int(t)+2]
                p4 = waypoints[int(t)+3]
                
                t = t % 1
                tt = t*t
                ttt = tt*t

                q1 = -ttt + 2.0*tt - t
                q2 = 3.0*ttt - 5.0*tt + 2.0
                q3 = -3.0*ttt + 4.0*tt + t
                q4 = ttt - tt

                tx = 0.5 * (p1[0] * q1 + p2[0] * q2 + p3[0] * q3 + p4[0] * q4);
                ty = 0.5 * (p1[1] * q1 + p2[1] * q2 + p3[1] * q3 + p4[1] * q4);
                
                total_waypoints.append((tx, ty))
                    
            for i in range(2, len(total_waypoints)):
                cv2.circle(img, (int(start_pos[0]+total_waypoints[i][0]),
                                int(start_pos[1]-total_waypoints[i][1])),
                        2, (150, 0,
                            255), -1)
                cv2.line(img, (int(start_pos[0]+total_waypoints[i][0]),
                                int(start_pos[1]-total_waypoints[i][1])),
                        (int(start_pos[0] + total_waypoints[i-1][0]),
                        int(start_pos[1] - total_waypoints[i-1][1])),
                        (150, 0,
                            255), 1)
            
        cv2.imshow("Field", img)
        key = cv2.waitKey()
        
        if __name__ == "__main__":
            if key == 13: # enter key to confirm
                if len(control_points) >= 4:
                    with open(config["PATH"]["FILE_LOCATION"], "w+") as file:
                        for w in total_waypoints:
                            file.write(str(w[0]*scaler) + "," + str(-w[1]*scaler) + "\n")
            
                with open(config["CONTROL_POINTS"]["FILE_LOCATION"], "w+") as file:
                    file.write(str(start_pos[0]*scaler) + "," + str(-start_pos[1]*scaler) + "\n")
                    for w in control_points:
                        file.write(str(w[0]*scaler) + "," + str(-w[1]*scaler) + "\n")
                isdeleting = False
                return []
        else:
            if key == 13: # enter key to confirm
                if len(control_points) >= 4:
                    with open(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + "/paths/" + f"{pathNo}.csv", "w+") as file:
                        for w in total_waypoints:
                            file.write(str(w[0]*scaler) + "," + str(-w[1]*scaler) + "\n")
            
                with open(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + "/control_points/" + f"{pathNo}.csv", "w+") as file:
                    file.write(str(start_pos[0]*scaler) + "," + str(-start_pos[1]*scaler) + "\n")
                    for w in control_points:
                        file.write(str(w[0]*scaler) + "," + str(-w[1]*scaler) + "\n")
                        
                with open(config["STRATEGY"]["FOLDER_LOCATION"] + strat_name + "/coordinates/" + f"{pathNo}.csv", "w+") as file:
                    for w in control_points:
                        file.write(str((start_pos[0] + w[0])*scalerPCT) + "," + str((start_pos[1] - w[1])*scalerPCT) + "\n")
                pathNo += 1
                isdeleting = False
                return [(start_pos[0] + control_points[len(control_points)-2][0]), -(start_pos[1] - control_points[len(control_points)-2][1])]

if __name__ == "__main__":
    generate_path([], "")