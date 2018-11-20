from pyCreate2 import create2
import lab11_map
import math
import random
from rrt_two import RRT
import numpy as np
import odometry
import pid_controller


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.map = lab11_map.Map("configuration_space.png")
        self.T = None
        self.start = (50, 250)
        self.end = (150, 50)
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.waypoints = [
            [150, 50],
            [150.14869758889074, 50.40240062947044],
            [151.38443594068957, 44.531033605863975],
            [157.34395035447554, 43.83519745282403],
            [159.9830566644057, 49.22362188719136],
            [165.01971207958653, 52.48431233995281],
            [166.08200989749037, 51.76595509541835],
            [162.8274040002357, 56.80654439167679],
            [162.58914539688016, 62.80181192848672],
            [161.65134293979207, 68.7280693099611],
            [166.24511621483907, 72.5877645178064],
            [169.4596961251666, 77.65397375155491],
            [169.81189729262783, 83.64362769294743],
            [168.1030136923147, 89.39512461466971],
            [169.13222641008716, 95.30619230008624],
            [164.28476945770043, 98.84203181776199],
            [159.13657223555904, 101.92360047056458],
            [153.29153438432476, 103.27841873056795],
            [153.2062807131846, 109.2778130176237],
            [147.20661026888413, 109.3406983083301],
            [142.43545544316433, 112.9788409153496],
            [138.14598394596814, 117.17412806580087],
            [134.00098880255507, 121.51221696081929],
            [128.50635958940123, 123.92241400877123],
            [124.88953147621035, 128.70974671077053],
            [122.03362683798868, 133.98647000249088],
            [124.28370096943293, 139.54858887959207],
            [126.46223063229577, 145.1391173437023],
            [130.15695556054735, 149.86659105643264],
            [127.60995902855399, 155.2991603073501],
            [132.9736448370284, 157.98818885832725],
            [133.45711508360725, 163.96867851392176],
            [131.07296807919232, 169.4746609931292],
            [131.24690427428683, 175.4721393132264],
            [134.99794743982267, 180.15505242050128],
            [138.87993530319093, 184.7300027184457],
            [136.02415090903725, 190.00679108771996],
            [130.8470605748313, 193.03956797886627],
            [126.64751896605162, 197.32487430118775],
            [125.42618081893612, 203.1992536837609],
            [124.30401895723566, 209.09338235186317],
            [123.03868731186792, 214.95844295063202],
            [123.80377133289849, 220.9094635724047],
            [121.8817393194328, 226.59328208984638],
            [115.88754582646332, 226.32937989424667],
            [110.90414570516865, 222.98786478413865],
            [105.27395281619683, 225.06173267601872],
            [99.30694346426476, 224.43340203538813],
            [93.47788328129653, 223.01141139430277],
            [89.13971178359841, 227.15632008528254],
            [83.2197041879971, 226.1798416840761],
            [77.4208438305826, 224.63931626876916],
            [72.91510466462213, 228.60142613200236],
            [66.9985043843938, 229.59834193787665],
            [62.60566001766925, 233.685262339565],
            [56.640797678751746, 234.3336582662669],
            [57.08162675626823, 240.3174421631348],
            [53.414063993062086, 245.06601914765804],
            [50, 250]
        ]

    def run(self):
        # This is an example on how to check if a certain point in the given map is an obstacle
        # Each pixel corresponds to 1cm
        # print(self.map.has_obstacle(50, 60))

        self.generate_rrt(self.start, 3000, 10.0)
        self.draw_edges()
        self.draw_shortest(self.T.print_shortest())
        self.follow_path()

        # This is an example on how to draw a line
        # self.map.draw_line((0, 0), (self.map.width, self.map.height), (255, 0, 0))
        # self.map.draw_line((0, self.map.height), (self.map.width, 0), (0, 255, 0))
        self.map.save("fp11_rrt.png")

    def draw_edges(self):
        for edge in self.T.edges:
            self.map.draw_line(edge[0], edge[1], color=(255, 0, 0))

    def draw_shortest(self, points):
        for i in range(len(points) - 1):
            self.map.draw_line(points[i], points[i+1], color=(0, 255, 0))

    @staticmethod
    def dist(p1, p2):
        return math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

    def random_state(self):
        while True:
            rand = random.random() * 300.0, random.random() * 300.0
            if not self.map.has_obstacle(rand[0], rand[1]):
                break
        return rand

    def nearest_neighbor(self, rand, nodes):
        nn = nodes[0]
        for p in nodes:
            if self.dist(p, rand) < self.dist(nn, rand):
                nn = p
        return nn

    def new_state(self, p1, p2, delta_t):
        if self.dist(p1, p2) < delta_t:
            return p2
        else:
            while True:
                theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                new_point = p1[0] + delta_t * math.cos(theta), p1[1] + delta_t * math.sin(theta)
                if not self.map.has_obstacle(new_point[0], new_point[1]):
                    break
                else:
                    p2 = self.random_state()
            return new_point

    def generate_rrt(self, x_init, K, delta_t):
        # x_init = starting coords (in lab) [270,310]
        # K = number of iterations = tuning ad hoc [2000]
        # delta_t = step size which is how much you wanna move, tune ad hoc
        self.T = RRT(x_init, self.end)
        for k in range(K):
            x_rand = self.random_state()
            x_near = self.nearest_neighbor(x_rand, self.T.nodes)
            x_new = self.new_state(x_near, x_rand, delta_t)
            self.T.add_vertex(x_new)
            self.T.add_edge(x_near, x_new)
        x_near = self.nearest_neighbor(self.end, self.T.nodes)
        x_new = self.new_state(x_near, self.end, delta_t)
        self.T.add_vertex(x_new)
        self.T.add_edge(x_near, x_new)

    def follow_path(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        self.waypoints.reverse()
        self.odometry.x = self.waypoints[0][1]/100
        self.odometry.y = self.waypoints[0][0]/100
        for waypoint in self.waypoints:
            goal_x = waypoint[1] / 100
            goal_y = -waypoint[0] / 100

            print("waypoint:", goal_x, goal_y)

            while True:
                state = self.create.update()
                if state is not None:
                    if abs(goal_x - self.odometry.x) <= .1 and abs(goal_y - self.odometry.y) <= .1:
                        break
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    # improved version 2: fuse with velocity controller
                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))


