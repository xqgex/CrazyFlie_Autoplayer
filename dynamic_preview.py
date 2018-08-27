#!/usr/bin/python3

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider
from matplotlib import animation

obs_lines = []
end_lines = []
i = 0
pathLength = 0


def load_sites(filename):
    with open(filename, 'r') as myfile:
        data = myfile.read().replace('\n', ' ')
    s = data.split()
    j = 0
    sites = []
    n = int(s[j])
    j += 1
    for i in range(n):
        sites.append([float(s[j]), float(s[j + 1])])
        j += 2
    return sites


def draw_sites(sites, ax):
    for point in sites:
        xs, ys = zip(*sites)  # create lists of x and y values
        xs = list(xs)
        xs = [x+0.5 for x in xs]
        ys = list(ys)
        ys = [y + 0.5 for y in ys]
        ax.plot(xs, ys, 'ro')


def load_obstacles(filename):
    with open(filename, 'r') as myfile:
        data = myfile.read().replace('\n', ' ')
    s = data.split()
    obs_num = int(s[0])
    res = []
    j = 1
    for i in range(obs_num):
        p_num = int(s[j])
        j += 1
        pol = []
        for k in range(p_num):
            pol.append([float(s[j]), float(s[j + 1])])
            j += 2
        pol.append(pol[0])  # repeat the first point to create a 'closed loop'
        res.append(pol)
    return res


def draw_obstacles(obstacles, ax):
    for pol in obstacles:
        line1, = ax.plot([], [], 'brown', lw=2)
        obs_lines.append(line1)


def draw_ends(end1, end2, ax):
    def cross(p, style):
        line1, = ax.plot([p[0] - 0.3, p[0] + 0.3], [p[1] + 0.3, p[1] - 0.3], style, lw=2)
        line2, = ax.plot([p[0] - 0.3, p[0] + 0.3], [p[1] - 0.3, p[1] + 0.3], style, lw=2)
        end_lines.append(line1)
        end_lines.append(line2)

    cross([x + 0.5 for x in end1], 'green')
    cross([x + 0.5 for x in end2], 'red')


def draw_path(path, ax):
    x1 = [t[0] + 0.5 for t in path]
    y1 = [t[1] + 0.5 for t in path]

    line = ax.plot(x1, y1, color='green', marker='.')
    return line


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
    i += 1
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
        path.append((float(data[i]), float(data[i + 1])))
        i += 2
    return path


if len(sys.argv) < 3:
    print("[USAGE1] sites.txt obstacles.txt output.txt")
    exit()

cmds = sys.argv[1:]
data = parse_file(cmds[0])
i = 0
obs = load_obstacles(cmds[1])
data = parse_file(cmds[2])

sites = load_sites(cmds[0])


i = 0
path = parse_path()

fig = plt.figure()

if not len(obs):
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-100, 100), ylim=(-100, 100))
else:
    xmin = min((min((x for (x, _) in poly)) for poly in obs))
    xmax = max((max((x for (x, _) in poly)) for poly in obs))
    ymin = min((min((y for (_, y) in poly)) for poly in obs))
    ymax = max((max((y for (_, y) in poly)) for poly in obs))
    abs_max = max(xmax, ymax)
    abs_min = min(xmin, ymin)
    margin = max(xmax - xmin, ymax - ymin) * 0.5
    #margin = max(xmax - xmin, ymax - ymin) * 1.5
    # margin = (abs_max - abs_min) * 1.5
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(xmin - margin, xmax + margin),
                         ylim=(ymin - margin, ymax + margin))

draw_sites(sites, ax)
draw_obstacles(obs, ax)
path_lines = draw_path(path, ax)
path_num_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
robot, = ax.plot([], [], 'o-', lw=2, color='black')
frames_per_path_part = 3000


def init():
    """initialize animation"""
    global obs, obs_lines
    ret = [robot, path_num_text]
    robot.set_data([], [])
    i = 0
    for pol in obs:
        xs, ys = zip(*pol)  # create lists of x and y values
        obs_lines[i].set_data(xs, ys)
        ret.append(obs_lines[i])
        i += 1

    path_num_text.set_text('')
    return tuple(ret)


currentFrame = 0
currentSpeed = 100.0


def animate(i):
    """perform animation step"""
    global path, frames_per_path_part, obs_lines, currentFrame, currentSpeed, pathLength, path_lines

    currentFrame = (currentFrame + currentSpeed) % (frames_per_path_part * (pathLength - 1))
    index = int(currentFrame / frames_per_path_part)
    part_of = currentFrame % frames_per_path_part
    part_of = part_of / frames_per_path_part
    p11 = path[index]
    p12 = path[index + 1]
    xs = [0, 0, 1, 1, 0]
    ys = [0, 1, 1, 0, 0]
    x1 = p11[0] + (p12[0] - p11[0]) * part_of
    x1 = [x1 + i for i in xs]
    y1 = p11[1] + (p12[1] - p11[1]) * part_of
    y1 = [y1 + i for i in ys]
    robot.set_data(x1, y1)
    ret = [robot, path_num_text]
    ret.extend(end_lines)

    for pol in obs_lines:
        ret.append(pol)

    for obj in path_lines:
        ret.append(obj)

    path_num_text.set_text('path part = %d -> %d' % (index, index + 1))
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
    currentSpeed = val if val <= 100 else 100 + (val - 100) ** 1.6181


splayer = Slider(axPlayer, 'Player', 0, frames_per_path_part * (pathLength - 1), valinit=currentFrame)
splayerSpeed = Slider(axSpeed, 'Speed', 0, 1000, valinit=currentSpeed)
splayer.on_changed(sliderChange)
splayerSpeed.on_changed(sliderChangeSpeed)

plt.show()
