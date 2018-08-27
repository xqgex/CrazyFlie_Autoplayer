import sys
import matplotlib.pyplot as plt



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


def draw_obstacles(obstacles):
	for pol in obstacles:
		xs, ys = zip(*pol)  # create lists of x and y values
		plt.plot(xs, ys, 'red')


def draw_robot(start, robot):
	xs, ys = zip(*robot)  # create lists of x and y values
	xs = list(xs)
	ys = list(ys)
	for i in range(len(xs)):
		xs[i] = xs[i] - robot[0][0] + start[0]
	for i in range(len(ys)):
		ys[i] = ys[i] - robot[0][1] + start[1]
	plt.plot(xs, ys, 'black')


def load_paths(filename):
	with open(filename, 'r') as myfile:
		data = myfile.read().replace('\n', ' ')
	s = data.split()
	j = 0
	path = []
	n = int(s[j])
	j += 1
	for i in range(n):
		path.append([float(s[j]), float(s[j + 1])])
		j += 2

	return path



def draw_path(path):
	for point in path:
		xs, ys = zip(*path)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		plt.plot(xs, ys, 'green')



def old_draw_path(path, robot):
	for point in robot:
		xs, ys = zip(*path)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		for i in range(len(xs)):
			xs[i] = xs[i] - robot[0][0] + point[0]
		for i in range(len(ys)):
			ys[i] = ys[i] - robot[0][1] + point[1]
		plt.plot(xs, ys, 'green')
	for point in path:
		xs, ys = zip(*robot)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		for i in range(len(xs)):
			xs[i] = xs[i] - robot[0][0] + point[0]
		for i in range(len(ys)):
			ys[i] = ys[i] - robot[0][1] + point[1]
		plt.plot(xs, ys, 'green')


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


def draw_sites(sites):
	for point in sites:
		xs, ys = zip(*sites)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		plt.plot(xs, ys, 'ro')


def load_robot(filename):
	with open(filename, 'r') as myfile:
		data = myfile.read().replace('\n', ' ')
	s = data.split()
	start = [float(s[0]), float(s[1])]
	n = int(s[2])
	res = []
	j = 3
	for i in range(n):
		res.append([float(s[j]), float(s[j + 1])])
		j += 2
	res.append(res[0])
	return [start, res]


if len(sys.argv) < 4:
	print("[USAGE1] RobotFile SitesFile ObstaclesFile")
	print("[USAGE2] RobotFile SitesFile ObstaclesFile OutputFile")
	exit()

cmds = sys.argv[1:]
plt.figure()
sites = load_sites(cmds[1])
draw_sites(sites)
obstacles = load_obstacles(cmds[2])
draw_obstacles(obstacles)
start, robot = load_robot(cmds[0])
if len(cmds) == 4:
	path = load_paths(cmds[3])
	draw_path(path)
	#old_draw_path(path, robot)
draw_robot(start, robot)


plt.show()
