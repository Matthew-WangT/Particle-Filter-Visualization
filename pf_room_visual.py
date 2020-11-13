# In this exercise, try to write a program that
# will resample particles according to their weights.
# Particles with higher weights should be sampled
# more frequently (in proportion to their weight).

# Don't modify anything below. Please scroll to the
# bottom to enter your code.


from math import *
import random
import matplotlib.pyplot as plt
import matplotlib.lines as lines

landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size_x = 100.0
world_size_y = 50.0


# world_size = world_size_x

class robot:
    def __init__(self):
        # change here
        self.x = random.random() * world_size_x
        self.y = random.random() * world_size_y
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise = 0.0;
        self.sense_noise = 0.0;

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size_x:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size_y:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise = float(new_t_noise);
        self.sense_noise = float(new_s_noise);

    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')

            # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size_x  # cyclic truncate
        y %= world_size_y

        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, x):

        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):

        # calculates how likely a measurement should be

        prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def eval(r, p):
    sum = 0.0;
    for i in range(len(p)):  # calculate mean error
        dx = (p[i].x - r.x + (world_size_x / 2.0)) % world_size_x - (world_size_x / 2.0)
        dy = (p[i].y - r.y + (world_size_y / 2.0)) % world_size_y - (world_size_y / 2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))


'''
重写一个激光雷达机器人的类，继承自robot
'''


class robot_lidar(robot):
    def __init__(self):
        robot.__init__(self)
        # do not need it
        # self.lidar_noise = 0.0;
        # 感知范围
        self.lidar_dist = 20;
        self.lidar_maxagl = 60 * pi / 180;  # 单边最大探测范围60°
        self.lidar_view = 120;  # view scope ---- for robot_lidar.sense()

    # NOTE: now the sense_noise comes from the lidar
    # but function sense() have to change
    def sense(self, isReal=False):  # 将真实距离与传感器距离整合到一个函数
        # now the Z record the dist between robot and wall from lidar sensor
        # len(Z) = 120
        Z = []
        # theta record 4 corner angles of room, the order like the pic below
        # 1-----------0
        # -           -
        # -           -
        # 2-----------3
        theta = []
        theta[0] = atan2(world_size_y - self.y, world_size_x - self.x)
        theta[1] = atan2(world_size_y - self.y, 0 - self.x)
        theta[2] = atan2(0 - self.y, 0 - self.x)
        theta[3] = atan2(0 - self.y, world_size_x - self.x)
        # 0-120 degree
        beta = self.orientation - self.lidar_maxagl
        for angle in range(self.lidar_view):
            beta = (beta*180/pi + 1)/180*pi
            if theta[0] < beta < theta[1]:
                dist = (world_size_y - self.y) / sin(beta)
            elif theta[1] < beta < theta[2]:
                dist = (world_size_y - self.y) / sin(beta - 1 / 2 * pi)
            elif theta[2] < beta < theta[3]:
                dist = (world_size_y - self.y) / sin(beta - pi)
            elif theta[3] < beta < (theta[0] + pi):
                dist = (world_size_y - self.y) / sin(beta - 3 / 2 * pi)
        if not isReal:
            dist += random.gauss(0.0, self.sense_noise)
        Z.append(dist)
        return Z

    '''
    # real distance without noise
    def real_dist(self):
        # now the Z record the dist between robot and wall from ladar sensor
        # len(Z) = 120
        Z = []
        # theta record 4 corner angles of room, the order like the pic below
        # 1-----------0
        # -           -
        # -           -
        # 2-----------3
        theta = []
        theta[0] = atan2(world_size_y - self.y, world_size_x - self.x)
        theta[1] = atan2(world_size_y - self.y, 0 - self.x)
        theta[2] = atan2(0 - self.y, 0 - self.x)
        theta[3] = atan2(0 - self.y, world_size_x - self.x)
        # 0-120 degree
        beta = self.orientation - self.ladar_maxagl
        for angle in range(self.ladar_view):
            if theta[0] < beta < theta[1]:
                dist = (world_size_y - self.y)/sin(beta)
            elif theta[1] < beta < theta[2]:
                dist = (world_size_y - self.y)/sin(beta - 1/2*pi)
            elif theta[2] < beta < theta[3]:
                dist = (world_size_y - self.y)/sin(beta - pi)
            elif theta[3] < beta < (theta[0] + pi):
                dist = (world_size_y - self.y)/sin(beta - 3/2*pi)
        dist += random.gauss(0.0, self.sense_noise)
        Z.append(dist)
        return Z
    '''

    def measurement_prob(self, measurement):

        # calculates how likely a measurement should be
        prob = 1.0
        realView = self.sense(self, True)
        for i in range(len(measurement)):
            prob *= self.Gaussian(realView[i], self.sense_noise, measurement[i])
        return prob


'''
the funtion below is to visualized the whole progress
'''

'''
p:  resampled particles--p3
pr: particles after move
'''
def visualization(robot, step, p, pr, weights):
    plt.figure(step, figsize=(15., 7.5))
    plt.title('Particle filter, step ' + str(step))

    # draw coordinate grid for plotting
    grid = [0, world_size_x, 0, world_size_y]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size_x), 5)])
    plt.yticks([i for i in range(0, int(world_size_y), 5)])

    # draw particles
    for ind in range(len(p)):
        # particle
        circle = plt.Circle((p[ind].x, p[ind].y), 1., facecolor='#ffb266', edgecolor='#994c00', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(p[ind].x, p[ind].y, 2 * cos(p[ind].orientation), 2 * sin(p[ind].orientation), alpha=1.,
                          facecolor='#994c00', edgecolor='#994c00')
        plt.gca().add_patch(arrow)

    # draw resampled particles
    for ind in range(len(pr)):
        # particle
        circle = plt.Circle((pr[ind].x, pr[ind].y), 1., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(pr[ind].x, pr[ind].y, 2 * cos(pr[ind].orientation), 2 * sin(pr[ind].orientation), alpha=1.,
                          facecolor='#006600', edgecolor='#006600')
        plt.gca().add_patch(arrow)

    # fixed landmarks of known locations
    # for lm in landmarks:
    #     circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
    #     plt.gca().add_patch(circle)
    line = lines.Line2D([0, 0], [0, world_size_y], lw=5, color='black')
    plt.gca().add_line(line)
    line = lines.Line2D([0, 0], [world_size_x, 0], lw=5, color='black')
    plt.gca().add_line(line)
    line = lines.Line2D([world_size_x, 0], [world_size_x, world_size_y], lw=5, color='black')
    plt.gca().add_line(line)
    line = lines.Line2D([0, world_size_y], [world_size_x, world_size_y], lw=5, color='black')
    plt.gca().add_line(line)

    # robot's location
    circle = plt.Circle((robot.x, robot.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)

    # robot's orientation
    arrow = plt.Arrow(robot.x, robot.y, 2 * cos(robot.orientation), 2 * sin(robot.orientation), alpha=0.5,
                      facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)

    # robot's lidar light
    dist_lidar = robot.sense()
    for i in range(len(dist_lidar)):
        dx = dist_lidar[i]*cos((60+i)*pi/180)
        dy = dist_lidar[i]*sin((60+i)*pi/180)
        line = lines.Line2D([robot.x, robot.y], [robot.x + dx, robot.y + dy], lw=1, color='red')
        plt.gca().add_line(line)

    plt.savefig('figure_' + str(step) + '.png')
    # plt.show()
    plt.close()


####   ======================================================================== ####
myrobot = robot_lidar()
myrobot = myrobot.move(0.1, 5.0)

N = 1000
T = 20  # Leave this as 10 for grading purposes.

p = []
for i in range(N):
    r = robot()
    r.set_noise(0.05, 0.05, 5.0)
    p.append(r)

for t in range(T):
    myrobot = myrobot.move(0.1, 5.0)
    Z = myrobot.sense()

    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.1, 5.0))
    p = p2

    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))

    # resample
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3
    # enter code here, make sure that you output 10 print statements.
    print(eval(myrobot, p))
    visualization(myrobot, t + 1, p3, p2, w)
    # plt.show()
