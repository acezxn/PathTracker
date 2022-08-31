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

class Robot:
    def __init__(self, config):
        self.field_length = float(config["FIELD_IMAGE"]["FIELD_LENGTH"])
        self.img_dimension = float(config["FIELD_IMAGE"]["IMAGE_LENGTH"])
        self.scaler = self.field_length / self.img_dimension
        self.width = float(config["ROBOT"]["TRACKWIDTH"]) / self.scaler
        self.length = float(config["ROBOT"]["LENGTH"]) / self.scaler
        self.maxVChange = float(config["ROBOT"]["MAX_VEL_CHANGE"])
        self.maxVel = float(config["VELOCITY"]["MAX_VEL"])
        self.pos = (0,0)
        self.start_pos = (0,0)
        self.angle = 0.0
        
        # for exporting
        self.itt = 0
        self.imageNames = []