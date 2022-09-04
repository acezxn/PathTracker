'''
Make strategies and run them
'''

from PathGenerator import generate_path
from objects.Robot import Robot
import algorithms.PurePursuit as PurePursuit
import algorithms.Ramsete as Ramsete
import algorithms.SimpleMoveToPoint as SIMPLE

import os
import cv2
import math
import configparser
import time
import imageio
import tkinter as tk
from tqdm import tqdm

def decrease_brightness(img, value=30):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        v[True] -= value

        final_hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return img

strat_name = ""
mainMenuOption = ""
pathNo = 0

def makeStrategy():
    global strat_name
    
    window = tk.Tk()
    window.title("Create strategy")
    window.geometry("400x200")
    
    def set_strat_name():
        global strat_name
        strat_name = entry.get()
        window.destroy()

    def writeToFile():
        global strat_name, pathNo
        algo = var.get()
        actionFile = open(f"strats/{strat_name}/actions.csv", "a+")
        actionFile.write(f"{algo},{pathNo},0\n")
        actionFile.close()   
        print("written")
        pathNo += 1
        window.destroy()
        
    
    pos = []
    label = tk.Label(text="Input the strategy name: ")
    label.pack()
    
    entry = tk.Entry()
    entry.pack()

    button = tk.Button(window, text= "confirm", height = 2, width = 5, command=set_strat_name)
    button.pack()
    
    window.mainloop()
    # while strat_name == "":
    #     pass
    # strat_name = input("Input the strategy name: ")

    os.mkdir(f"strats/{strat_name}")
    os.mkdir(f"strats/{strat_name}/control_points")
    os.mkdir(f"strats/{strat_name}/paths")
    os.mkdir(f"strats/{strat_name}/coordinates")

    pathNo = 0

    while True:
        end_pos = generate_path(pos, strat_name)
        if len(end_pos) == 2:
            pos = [end_pos[0], -end_pos[1]]
            
            window = tk.Tk()


            label = tk.Label(text="what algo would you want the robot to follow this path (RAMSETE, PURE_PURSUIT, or SIMPLE): ")
            label.pack()

            algo_choices = ["SIMPLE", "PURE_PURSUIT", "RAMSETE"]
            var = tk.StringVar(window)
            var.set("SIMPLE")
            selection = tk.OptionMenu(window, var, *algo_choices)
            selection.pack()

            button = tk.Button(window, text= "confirm", height = 2, width = 5, command=writeToFile)
            button.pack()

            window.mainloop()
    
        else:
            break
        
    
def runStrategy():
    global strat_name
    
    def set_strat_name():
        global strat_name
        strat_name = entry.get()
        window.destroy()
    
    config = configparser.ConfigParser()
    config.read("config.ini")

    robot = Robot(config)


    field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
    img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])
    export_enabled = int(config["EXPORT"]["ENABLED"])
    imageNames = []

    scaler = field_length / img_dimension

    # parse actions.csv
    window = tk.Tk()
    window.title("Run strategy")
    window.geometry("400x200")
    
    label = tk.Label(text="Input the strategy name: ")
    label.pack()
    
    entry = tk.Entry()
    entry.pack()

    button = tk.Button(window, text= "confirm", height = 2, width = 5, command=set_strat_name)
    button.pack()
    
    window.mainloop()
    
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
    
    # field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
    # img = decrease_brightness(field, 100)
    
    # cv2.imshow("img", img)
    
    # cv2.waitKey(0) 
    
    # run actions.csv
    for k, command in enumerate(actions):
        if command[0] == "SIMPLE":
            path = []
            pathNo = command[1]
            pathFile = open(strat_dir + f"control_points/{pathNo}.csv", "r")
            for line in pathFile.readlines():
                inner = []
                for i in range(len(line.split(","))):
                    if i == 1:
                        inner.append(-float(line.split(",")[i])/scaler)
                    else:
                        inner.append(float(line.split(",")[i])/scaler)
                path.append(list(inner))
            robot.start_pos = path[0]
            print("turning", robot.angle)
            SIMPLE.run(robot, strat_name, [path[2],path[2]], True) 
            # SIMPLE.rotateToAngle(path[0], math.atan2(path[2][0], path[2][1]))
            
            SIMPLE.run(robot, strat_name, path, False)   
            if (k == len(actions)-1):    
                key = cv2.waitKey()
                    

        elif command[0] == "PURE_PURSUIT":
            path = []
            pathNo = command[1]
            pathFile = open(strat_dir + f"paths/{pathNo}.csv", "r")
            for line in pathFile.readlines():
                inner = []
                for i in range(len(line.split(","))):
                    if i == 1:
                        inner.append(-float(line.split(",")[i])/scaler)
                    else:
                        inner.append(float(line.split(",")[i])/scaler)
                path.append(list(inner))
            # SIMPLE.rotateToAngle(path[0], math.atan2(path[2][0], path[2][1]))
            robot.start_pos = path[0]
            SIMPLE.run(robot, strat_name, [path[2],path[2]], True) 

            PurePursuit.run(robot, strat_name, path)        
            if (k == len(actions)-1):    
                key = cv2.waitKey()

        elif command[0] == "RAMSETE":
            path = []
            pathNo = command[1]
            pathFile = open(strat_dir + f"paths/{pathNo}.csv", "r")
            for line in pathFile.readlines():
                inner = []
                for i in range(len(line.split(","))):
                    if i == 1:
                        inner.append(-float(line.split(",")[i])/scaler)
                    else:
                        inner.append(float(line.split(",")[i])/scaler)
                path.append(list(inner))
            # SIMPLE.rotateToAngle(path[0], math.atan2(path[2][0], path[2][1]))
            robot.start_pos = path[0]
            print("turning", robot.angle)
            SIMPLE.run(robot, strat_name, [path[2],path[2]], True) 

            Ramsete.run(robot, strat_name, path)     
            if (k == len(actions)-1):    
                key = cv2.waitKey()   
    
    if export_enabled:
        with imageio.get_writer('images/movie.gif', mode='I') as writer:
            print("Writing gif to images/movie.gif")
            for name in tqdm(robot.imageNames):
                writer.append_data(imageio.imread(name))
            print("Done!")

def mainMenuSelect():
        global mainMenuOption
        mainMenuOption = var.get()
        window.destroy()

def findCoordinate():
    global mouse_down, scaler
    
    def click(event, x, y, flags, param):
        global mouse_down, scaler
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_down = True
        if event == cv2.EVENT_LBUTTONUP and mouse_down:
            mouse_down = False
            window = tk.Tk()
            label = tk.Label(text=f"({x * scaler}, {y * scaler})")
            label.pack()
            window.mainloop()
            
            
    config = configparser.ConfigParser()
    config.read("config.ini")
    
    field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
    img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])
    export_enabled = int(config["EXPORT"]["ENABLED"])
    imageNames = []

    scaler = 100 / img_dimension
    
    img = decrease_brightness(field, 100)
    mouse_down = False
    
    cv2.imshow("img", img)
    cv2.setMouseCallback('img', click)
    
    cv2.waitKey(0) 
    


if __name__ == "__main__":
    window = tk.Tk()
    window.title("Main menu")
    window.geometry("400x400")
    
    label = tk.Label(text="PathTracker", font = ("Arial", 30))
    label.pack(pady = 100)
    
    options = ["Create a new strategy", "Run a strategy", "Find coordinates"]
    var = tk.StringVar(window)
    var.set("Create a new strategy")
    selection = tk.OptionMenu(window, var, *options)
    selection.config(width=len(max(options, key=len)))
    selection.pack(pady = 10, ipadx=10)
    
    button = tk.Button(window, text= "confirm", height = 2, width = 5, command=mainMenuSelect)
    button.pack()
    
    window.mainloop()

    if (mainMenuOption == options[0]):
        makeStrategy()
    elif (mainMenuOption == options[1]):
        runStrategy()
    elif (mainMenuOption == options[2]):
        findCoordinate()
    