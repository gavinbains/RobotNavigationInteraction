from pyCreate2 import create2
import lab11_map
import math
import random
from rrt import RRT
import numpy as np
import odometry
import pid_controller


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.map = lab11_map.Map("lab11.png")
        self.T = None
        self.start = (270, 310)
        self.end = (40, 120)
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.waypoints = [
            [40, 120],
            [39.28675370589847, 120.01932406495165],
            [38.062448893839715, 119.63995519285812],
            [36.76656799401991, 121.2046362727171],
            [37.157040212491864, 123.92775147537525],
            [42.36874934960837, 126.9006436480072],
            [45.33061574083489, 132.11862673350325],
            [42.04878434934366, 137.14153253769894],
            [42.107632283026746, 139.52535721269732],
            [46.44097227470616, 143.6753167928031],
            [49.956550418172476, 148.53748815107318],
            [51.52677635313172, 148.12719606540477],
            [57.155873775291695, 150.20403554206925],
            [62.968754699788335, 151.6907821101242],
            [64.32666925199392, 157.5351014091451],
            [67.61109987268236, 162.55630798703893],
            [68.2159126672996, 168.52574694589154],
            [72.71896165388014, 172.49091400724672],
            [78.71625437614071, 172.31069191226263],
            [84.38698684374737, 174.27099629700282],
            [86.07285569082867, 168.5127112892561],
            [89.88470993096207, 173.146259309076],
            [95.75836150240443, 171.92142576230617],
            [101.75763647948494, 171.82815331527343],
            [103.85115402220661, 166.20523731925172],
            [109.66252302272297, 164.71259190260463],
            [111.43988897096078, 158.98188773243106],
            [114.52895401282117, 153.83818506147932],
            [115.04642268926384, 147.86054119713774],
            [120.61387481659224, 145.6236959164117],
            [122.61863170107345, 139.96852573027888],
            [128.61721712123494, 139.83824542852142],
            [131.0725506951374, 134.363637036968],
            [137.05105421807096, 133.85619676829492],
            [140.9763741966022, 129.3183709600313],
            [144.42127211733043, 124.4058687141527],
            [147.77481666970738, 119.43055576568173],
            [153.76032379273013, 119.01377707111594],
            [157.183000589001, 114.08576697085549],
            [163.102313945438, 113.10508894606518],
            [167.03284067465762, 108.57177232013996],
            [171.10890288287948, 104.16885091785423],
            [176.3125568925019, 107.15582010888058],
            [181.84127851512596, 109.48675065686544],
            [186.41909746730238, 105.60814605712928],
            [192.35106219423622, 104.70714967621944],
            [197.58887950233185, 101.78050144880743],
            [202.92166642631773, 104.53029846374604],
            [208.8817602182883, 105.22115444795253],
            [214.49326273946167, 107.34507522210228],
            [220.47280476264166, 106.85002250468276],
            [226.4571219624726, 106.41649320716347],
            [231.8894089118846, 108.96409178214505],
            [237.83676904589257, 109.7571286264422],
            [237.0130815645053, 115.70032128132105],
            [242.50458603877925, 118.11762941414759],
            [244.85464795947144, 123.63824613412666],
            [250.84910259908935, 123.8961483899589],
            [252.87583265235864, 129.54348097191075],
            [255.63848384276662, 134.86962013645925],
            [259.11526488070604, 139.75960925855603],
            [260.7667326083286, 145.52785460221307],
            [259.15902903100977, 151.30845052369696],
            [256.5113312401975, 156.6926585813901],
            [253.77318310895174, 162.03143604550635],
            [253.01698352490092, 167.9835921413326],
            [252.2747920916053, 173.9375111764172],
            [248.0919286743255, 178.23909851510766],
            [242.09250360242333, 178.32215757019407],
            [236.397924787015, 180.2120708261671],
            [230.42457011639235, 179.64723989663992],
            [231.58401467607234, 173.76033168323718],
            [225.72552966526695, 175.05576727722413],
            [219.72885773816682, 175.25558168793887],
            [213.9176578846473, 176.74888549576053],
            [211.25012736164467, 182.12329531534533],
            [213.46846635597922, 187.6981470717793],
            [208.75977823908775, 191.41678341025676],
            [212.1020544563171, 196.39967310276882],
            [207.84549177349166, 200.6283459592544],
            [210.2256813860439, 206.13604034150714],
            [205.48820990503572, 209.81793713074964],
            [211.40089473898203, 210.83781852307973],
            [211.3748670277366, 216.83776206933476],
            [211.35188922150957, 222.83771807087518],
            [216.89836820472493, 225.1260735696451],
            [221.43819623510018, 229.04907770609705],
            [224.04396699518205, 234.4537015362912],
            [227.19878805656901, 239.55733789389242],
            [232.72709225685315, 237.2254175116493],
            [238.71790949072218, 237.55724407836416],
            [244.6933278162382, 238.09980820829463],
            [247.91218307673483, 243.1633021415361],
            [253.04239353012065, 246.27472315193808],
            [258.44957472674497, 248.87518301643752],
            [259.1587861899647, 254.83312050355818],
            [264.6031754091133, 257.3547521242392],
            [264.63023864694566, 263.35469108902527],
            [265.0752789726481, 269.33816325061326],
            [264.51063457429154, 275.31153555638866],
            [264.1428556634168, 281.30025317143077],
            [264.3847558062311, 287.29537488169706],
            [264.04495631945935, 293.28574517983304],
            [266.78980487929175, 298.62108081909565],
            [267.40696072157334, 304.58925630799826],
            [270, 310]
        ]

    def run(self):
        # This is an example on how to check if a certain point in the given map is an obstacle
        # Each pixel corresponds to 1cm
        # print(self.map.has_obstacle(50, 60))

        self.generate_rrt(self.start, 3000, 6.0)
        self.draw_edges()
        self.draw_shortest(self.T.print_shortest())
        self.follow_path()

        # This is an example on how to draw a line
        # self.map.draw_line((0, 0), (self.map.width, self.map.height), (255, 0, 0))
        # self.map.draw_line((0, self.map.height), (self.map.width, 0), (0, 255, 0))
        self.map.save("lab11_rrt.png")

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
            rand = random.random() * 300.0, random.random() * 335.0
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
        self.odometry.x = -self.waypoints[0][1]/100
        self.odometry.y = -self.waypoints[0][0]/100
        for waypoint in self.waypoints:
            goal_x = -waypoint[1] / 100
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


