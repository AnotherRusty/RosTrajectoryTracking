#!/usr/bin/env python


#  *  Copyright (c) 2019, 2030, Pibot Technology (Suzhou) Co., Ltd.
#  *  All rights reserved.
#  *
#  *  Redistribution and use in source and binary forms, with or without
#  *  modification, are permitted provided that the following conditions
#  *  are met:
#  *
#  *   * Redistributions of source code must retain the above copyright
#  *     notice, this list of conditions and the following disclaimer.
#  *   * Redistributions in binary form must reproduce the above
#  *     copyright notice, this list of conditions and the following
#  *     disclaimer in the documentation and/or other materials provided
#  *     with the distribution.
#  *   * Neither the name of Pibot Technology (Suzhou) Co., Ltd. nor the
#  *     names of its contributors may be used to endorse or promote 
#  *     products derived from this software without specific prior 
#  *     written permission.
#  *
#  * Author: Danny Zhu @Pibot

'''
    run this script to genrate path yaml file
'''

import pygame
import yaml
from math import sqrt

PATH_NAME = 'test_path'
MAP_NAME = 'test_map'

PATH_COLOR = (0, 255, 0)    # green

def get_pos():
    return pygame.mouse.get_pos()

def convert(p):
    # pygame coordinate:  (0,0) top left
    # should transform to (0,0) bottom left for ROS map coordinate
    (px, py) = p
    rx = px
    ry = h -py
    return (rx, ry)

def distance(a, b):
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def draw_path(start, end):
    pygame.draw.line(screen, PATH_COLOR, start, end, 1)

def clear_path():
    screen.blit(map, (0,0))
    fpath['global_path']['waypoints'] = []

def save_path():
    save_name = raw_input('name the path: ')
    fpath['global_path']['name'] = save_name
    print "saving path to path/"+save_name+".yaml"
    with open("../path/"+save_name+".yaml", 'w') as f:
        yaml.dump(fpath, f, default_flow_style=True)


with open("../path/default.yaml", 'r') as f:
    fpath = yaml.safe_load(f)

print fpath

pygame.init()
map = pygame.image.load("../maps/"+MAP_NAME+".pgm")
w, h = map.get_size()
screen = pygame.display.set_mode((w,h))

screen.blit(map, (0,0))
done = False
drawing = False

while not done:
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
                if not drawing:
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                        save_path()
                        done = True
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        clear_path()
                        start_point = get_pos()
                        fpath['global_path']['waypoints'].append(convert(start_point))
                        drawing = True
                if drawing:
                    if event.type == pygame.MOUSEBUTTONUP:
                        drawing = False
        
        if drawing:
            end_point = get_pos()
            if(distance(start_point, end_point) > 1):
                draw_path(start_point, end_point)
                fpath['global_path']['waypoints'].append(convert(end_point))
                start_point = end_point

        pygame.display.flip()