#!/usr/bin/env python

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
screen = pygame.display.set_mode(map.get_size())

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
                        start_point = get_pos()
                        fpath['global_path']['waypoints'].append(start_point)
                        clear_path()
                        drawing = True
                if drawing:
                    if event.type == pygame.MOUSEBUTTONUP:
                        drawing = False
        
        if drawing:
            end_point = get_pos()
            if(distance(start_point, end_point) > 1):
                draw_path(start_point, end_point)
                fpath['global_path']['waypoints'].append(end_point)
                start_point = end_point

        pygame.display.flip()