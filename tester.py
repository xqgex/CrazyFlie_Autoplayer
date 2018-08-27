import sys
from subprocess import Popen, PIPE
from os import listdir
from os.path import isfile, join
from filecmp import cmp
import time

cmds = sys.argv[1:]

dir = 'tests'
print("run tests")
robots = [join(dir, f) for f in listdir(dir) if isfile(join(dir, f)) and join(dir, f).startswith(dir + '/robot')]
obstacles = [join(dir, f) for f in listdir(dir) if isfile(join(dir, f)) and join(dir, f).startswith(dir + '/obstacles')]

robots.sort(key=lambda s: int(s.split('robot')[1]))
obstacles.sort(key=lambda s: int(s.split('obstacles')[1]))

for test in zip(robots, obstacles):
    if len(cmds) != 0 and not test[0].split('robot')[1] in cmds:
        continue

    print('test %s' % str(test))

    p = Popen(['./PathFinder', test[0], test[1], 'output1'], stdin=PIPE, stdout=PIPE, stderr=PIPE)
    output, err = p.communicate()
    time.sleep(0.1)

    if p.returncode != 0:
        print('FAILED %s' % str(err.decode()))
        print('error code %d' % p.returncode)
        continue

    lines = output.decode().split('\n')
    for line in lines:
        if 'Path validation:' in line:
            if 'Success' in line:
                print('PASSED')
            else:
                try:  # sometimes there is a failure reason:
                    print('FAILED %s' % line.split('FAILURE:',1)[1])
                except:
                    print('FAILED')
            p = Popen(['python3', 'PreviewPy.py', test[0], test[1], 'output1'], stdin=PIPE, stdout=PIPE, stderr=PIPE)
            output, err = p.communicate()
            time.sleep(1)
            break
        elif 'ERROR:' in line:
            try:  # sometimes there is a failure reason:
                print('ERROR: %s' % line.split('ERROR:', 1)[1])
            except:
                print('ERROR')

            p = Popen(['python3', 'PreviewPy.py', test[0], test[1]], stdin=PIPE, stdout=PIPE, stderr=PIPE)
            output, err = p.communicate()
            time.sleep(1)
            break
