#!/usr/bin/env python

import cv2, pygame, yaml
from PIL import Image
import numpy as np

class map:
    def __init__(self):
        self.obstracle_x = []
        self.obstracle_y = []

    def map_read(self, jpg_path, yaml_path):
        self.map_image = Image.open(jpg_path)
        self.map = pygame.image.fromstring(self.map_image.tobytes(), self.map_image.size, self.map_image.mode)
        self.map_info = yaml.safe_load(open(yaml_path))
        w, h = self.map_image.size
        return w, h, self.map_info
        
    def set_map_obstacle(self):
        one_ch_image = self.map_image.convert('1')
        image = one_ch_image.load()
        w, h = one_ch_image.size
        for y in range(h):
            for x in range(w):
                if image[x, y] == 0:
                    self.obstracle_x.append(x)
                    self.obstracle_y.append(y)

    def on_linear_function(self, m, b):
        sol_x, sol_y = [], []
        for cnt in range(len(self.obstracle_x)):
            y = m*self.obstracle_x[cnt] + b
            Y = [ int(np.ceil(y)), int(np.trunc(y)) ]
            if self.obstracle_y[cnt] in Y:
                sol_x.append(self.obstracle_x[cnt])
                sol_y.append(self.obstracle_y[cnt])
        return sol_x, sol_y

    def get_obstracle(self):
        self.set_map_obstacle()
        return (self.obstracle_x, self.obstracle_y)

    def get_map(self):
        return self.map

    def get_map_info(self):
        return self.map_info
        
