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
        self.start = (100, 250)
        self.end = (150, 50)
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

        self.waypoints = []
        # self.waypoints = [
        #     [150, 50],
        #     [148.1839902399901, 48.77721068889793],
        #     [147.7456873472306, 49.35175670074556],
        #     [143.78340214947022, 53.532906658401366],
        #     [152.38281511349754, 54.56386388866407],
        #     [155.91401384634196, 61.194162234571095],
        #     [155.3152360746851, 62.172473664390296],
        #     [160.16587321713737, 70.91726178716733],
        #     [152.8941391535281, 77.78179991497642],
        #     [156.18607684046853, 87.22442382283705],
        #     [156.6847270858388, 97.21198348137187],
        #     [160.61438360276432, 106.40751498634671],
        #     [155.3610205782822, 113.87879448081841],
        #     [146.79872758494318, 119.04475409121197],
        #     [138.24978803008443, 124.23278189446935],
        #     [133.82168999117832, 133.19893757892163],
        #     [140.28720242575596, 140.82764316757306],
        #     [133.80538205759078, 148.44249738251628],
        #     [134.08244016092655, 158.43865858606716],
        #     [126.78615495208547, 165.27709569927123],
        #     [123.55470953772019, 174.74059191018492],
        #     [130.3842200285745, 182.04523334254222],
        #     [125.53886739510159, 190.79295063663085],
        #     [124.4628990102248, 200.73489672553848],
        #     [122.27403057667186, 210.49239922262223],
        #     [117.92059958103229, 219.4950454206134],
        #     [111.3921763086509, 227.06998367624487],
        #     [104.0249832511163, 233.83197054794272],
        #     [94.4635437045154, 230.9029990908103],
        #     [96.19597383721772, 240.75179017577628],
        #     [100, 250]
        # ]

    def run(self, points):

        x = points[0]
        y = abs(3 - points[1])

        print("The values to be computed are", x, " and ", y)

        self.start = (x, y)
        self.generate_rrt(self.start, 3000, 10.0)
        self.draw_edges()

        self.waypoints = self.T.print_shortest()
        self.draw_shortest(self.waypoints)

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
        self.odometry.x = self.waypoints[0][0]/100
        self.odometry.y = 3 - self.waypoints[0][1]/100
        for waypoint in self.waypoints:
            goal_x = waypoint[0] / 100
            goal_y = 3 - waypoint[1] / 100

            print("waypoint:", goal_x, goal_y)

            while True:
                state = self.create.update()
                if state is not None:
                    if abs(goal_x - self.odometry.x) <= .2 and abs(goal_y - self.odometry.y) <= .2:
                        break
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    # improved version 2: fuse with velocity controller
                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))


