import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def load_obstacles(filename):
	with open(filename, 'r') as myfile:
		data = myfile.read().replace('\n', ' ')
	s = data.split()
	n = int(s[0])
	res = []
	j = 1
	for i in range(n):
		n1 = int(s[j])
		j += 1
		pol = []
		for k in range(n1):
			pol.append([float(s[j]), float(s[j + 1])])
			j += 2
		pol.append(pol[0])  # repeat the first point to create a 'closed loop'
		res.append(pol)
	return res


def draw_obstacles(obstacles):
	for pol in obstacles:
		xs, ys = zip(*pol)  # create lists of x and y values
		plt.plot(xs, ys, 'brown')


def draw_robot(start, robot):
	xs, ys = zip(*robot)  # create lists of x and y values
	xs = list(xs)
	ys = list(ys)
	for i in range(len(xs)):
		xs[i] = xs[i] - robot[0][0] + start[0]
	for i in range(len(ys)):
		ys[i] = ys[i] - robot[0][1] + start[1]
	plt.plot(xs, ys, 'blue')


def load_paths(filename):
	with open(filename, 'r') as myfile:
		data = myfile.read().replace('\n', ' ')
	s = data.split()
	j = 0
	good_path = []
	n = int(s[j])
	j += 1
	for i in range(n):
		good_path.append([float(s[j]), float(s[j + 1])])
		j += 2
	bad_path = []
	n = int(s[j])
	j += 1
	for i in range(n):
		bad_path.append([float(s[j]), float(s[j + 1])])
		j += 2
	bad_obstacles = []
	n = int(s[j])
	j += 1
	for i in range(n):
		bad_obstacles.append(int(s[j]))
		j += 1

	return [good_path, bad_path, bad_obstacles]


def draw_good_path(good_path, robot):
	if not good_path:
		return
	for point in robot:
		xs, ys = zip(*good_path)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		for i in range(len(xs)):
			xs[i] = xs[i] - robot[0][0] + point[0]
		for i in range(len(ys)):
			ys[i] = ys[i] - robot[0][1] + point[1]
		plt.plot(xs, ys, 'green')
	for point in good_path:
		xs, ys = zip(*robot)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		for i in range(len(xs)):
			xs[i] = xs[i] - robot[0][0] + point[0]
		for i in range(len(ys)):
			ys[i] = ys[i] - robot[0][1] + point[1]
		plt.plot(xs, ys, 'green')


def draw_bad_path(bad_path, robot):
	if not bad_path:
		return
	for point in robot:
		xs, ys = zip(*bad_path)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		for i in range(len(xs)):
			xs[i] = xs[i] - robot[0][0] + point[0]
		for i in range(len(ys)):
			ys[i] = ys[i] - robot[0][1] + point[1]
		plt.plot(xs, ys, 'red')
	for point in bad_path:
		xs, ys = zip(*robot)  # create lists of x and y values
		xs = list(xs)
		ys = list(ys)
		for i in range(len(xs)):
			xs[i] = xs[i] - robot[0][0] + point[0]
		for i in range(len(ys)):
			ys[i] = ys[i] - robot[0][1] + point[1]
		plt.plot(xs, ys, 'red')


def draw_bad_obstacles(obstacles, bad_obstacles):
	for i in bad_obstacles:
		xs, ys = zip(*obstacles[i])  # create lists of x and y values
		plt.plot(xs, ys, 'magenta')


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


if len(sys.argv) < 2:
	print("[USAGE1] RobotFile ObstaclesFile")
	print("[USAGE2] RobotFile ObstaclesFile OutputFile")
	exit()

cmds = sys.argv[1:]
plt.figure()
obstacles = load_obstacles(cmds[1])
draw_obstacles(obstacles)
start, robot = load_robot(cmds[0])
if len(cmds) == 3:
	good_path, bad_path, bad_obstacles = load_paths(cmds[2])
	draw_good_path(good_path, robot)
	draw_bad_path(bad_path, robot)
	draw_bad_obstacles(obstacles, bad_obstacles)
draw_robot(start, robot)
patchs = [mpatches.Patch(color='red', label='paths that touched an obstacle'),
		  mpatches.Patch(color='magenta', label='obstacle that the path touched'),
		  mpatches.Patch(color='blue', label='the robot start point'),
		  mpatches.Patch(color='brown', label='obstacles'),
		  mpatches.Patch(color='green', label='paths that you handled correctly')]

plt.legend(handles=patchs)

plt.show()
