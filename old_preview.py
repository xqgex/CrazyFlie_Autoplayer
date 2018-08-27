#!/usr/bin/python3

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider
from matplotlib import animation

obs_lines = []
i=0
pathLength=0

def draw_obstacles(obstacles, ax):
    for pol in obstacles:
        line1, = ax.plot([], [], 'brown', lw=2)
        obs_lines.append(line1)

def parse_file(filename):
    with open(filename, 'r') as myfile:
        d = myfile.read().replace('\n', ' ')
        d = d.replace('\r', ' ')
        d = d.replace('\t', ' ')
        d = d.replace('  ', ' ')
        for ___ in range(10):
            d = d.replace('  ', ' ')
    return d.split()
    
    
def parse_polygon():
    global i, data
    ver_num = int(data[i])
    if ver_num == 0:
        return None
    i += 1
    poly = []
    for __ in range(ver_num):
        poly.append((float(data[i]), float(data[i + 1])))
        i += 2
    poly.append(poly[0])
    return poly    

def parse_obstacles():
    global i, data
    obs = []
    poly = parse_polygon()
    if poly:
        obs.append(poly)
    obs_num = int(data[i])
    i+=1
    for _ in range(obs_num):
        poly = parse_polygon()
        if poly:
            obs.append(poly)
    return obs
    
def parse_path():
    global i, data, pathLength
    path = []
    pathLength = int(data[i])
    i += 1
    for _ in range(pathLength):
        path.append(((float(data[i]), float(data[i + 1])), (float(data[i + 2]), float(data[i + 3]))))
        i += 4
    return path

if len(sys.argv) < 3:
    print("[USAGE1] obstacles.txt output.txt")
    exit()

cmds = sys.argv[1:]
data = parse_file(cmds[0])
i=0
obs = parse_obstacles()
data = parse_file(cmds[1])

i=0
path = parse_path()

fig = plt.figure()

ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-100, 100), ylim=(-100, 100))

draw_obstacles(obs, ax)
path_num_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
robot1, = ax.plot([], [], 'o-', lw=2,  color='green')
robot2, = ax.plot([], [], 'o-', lw=2,  color='red')
frames_per_path_part = 3000


def init():
    """initialize animation"""
    global obs, obs_lines
    ret = [robot1,robot2, path_num_text]
    robot1.set_data([], [])
    robot2.set_data([], [])
    i = 0
    for pol in obs:
        xs, ys = zip(*pol)  # create lists of x and y values
        obs_lines[i].set_data(xs, ys)
        ret.append(obs_lines[i])
        i += 1

    path_num_text.set_text('')
    return tuple(ret)


currentFrame = 0
currentSpeed = 20


def animate(i):
    """perform animation step"""
    global path, frames_per_path_part, obs_lines, currentFrame, currentSpeed, pathLength
    currentFrame = (currentFrame + currentSpeed) % (frames_per_path_part * (pathLength - 1))
    index = int(currentFrame / frames_per_path_part)
    part_of = currentFrame % frames_per_path_part
    part_of = part_of / frames_per_path_part
    p11, p21 = path[index]
    p12, p22 = path[index + 1]
    x1 = [p11[0] + (p12[0] - p11[0]) * part_of]
    y1 = [p11[1] + (p12[1] - p11[1]) * part_of]
    x2 = [p21[0] + (p22[0] - p21[0]) * part_of]
    y2 = [p21[1] + (p22[1] - p21[1]) * part_of]

    robot1.set_data(x1, y1)
    robot2.set_data(x2, y2)
    ret = [robot1, robot2, path_num_text]

    for pol in obs_lines:
        ret.append(pol)

    path_num_text.set_text('path part = %d -> %d (0-%d)' % (index, index + 1, len(path) - 1))
    return tuple(ret)


ani = animation.FuncAnimation(fig, animate, frames=frames_per_path_part * (pathLength - 1), interval=33, blit=True,
                              init_func=init)

axcolor = 'lightgoldenrodyellow'
plt.subplots_adjust(bottom=0.2)
axPlayer = plt.axes([0.1, 0.1, 0.8, 0.025], facecolor=axcolor)
axSpeed = plt.axes([0.1, 0.05, 0.8, 0.025], facecolor=axcolor)

def sliderChange(val):
    global currentFrame
    currentFrame = val


def sliderChangeSpeed(val):
    global currentSpeed
    currentSpeed = val


splayer = Slider(axPlayer, 'Player', 0, frames_per_path_part * (pathLength - 1), valinit=currentFrame)
splayerSpeed = Slider(axSpeed, 'Speed', 0, 1000, valinit=currentSpeed)
splayer.on_changed(sliderChange)
splayerSpeed.on_changed(sliderChangeSpeed)

plt.show()
