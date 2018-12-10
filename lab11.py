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

        self.map = lab11_map.Map("finalproject_map2_config.png")
        self.T = None

        self.start = (180, 268)

        self.end = (50, 150)
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(150, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(500, 0, 50, [0, 0], [-200, 200], is_angle=False)

        self.waypoints = []

    def run(self, points):

        self.odometry.x = points[0]
        self.odometry.y = points[1]
        self.odometry.theta = points[2]
        self.start = (points[0], points[1])
        print("Generating RRT")
        self.generate_rrt(self.start, 3000, 12.0)
        self.draw_edges()
        print("Drawing edges")
        self.waypoints = self.T.print_shortest()
        self.smooth_waypoints()

        # After smoothing out, calculate target theta
        print("Going to goal theta")
        tTheta = math.atan2((points[1] - self.waypoints[0][1]), (points[0] - self.waypoints[0][0]))
        self.draw_shortest(self.waypoints)
        self.pidTheta.update(points[2], tTheta, self.time.time())

        self.map.save("fp_rrt.png")

        print("waypoints: ", self.waypoints)

        self.follow_path()

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

    def smooth_waypoints(self):

        # smoothed_points = []
        #
        # add_point = True
        #
        # for waypoint in self.waypoints:

            # add_point = True
            #
            # x = waypoint[0] / 100
            # y = 3 - waypoint[1]/100
            #
            # # Check Up
            # y_up = y + 0.2
            #
            # if self.map.has_obstacle(x, y_up):
            #     add_point = False
            #
            # # Check Down
            # y_down = y - 0.2
            #
            # if self.map.has_obstacle(x, y_down):
            #     add_point = False
            #
            # # Check Right
            # x_right = x + 0.2
            #
            # if self.map.has_obstacle(x_right, y):
            #     add_point = False
            #
            # # Check Left
            # x_left = x - 0.2
            #
            # if self.map.has_obstacle(x_left, y):
            #     add_point = False
            #
            # if add_point:
            #     smoothed_points.append(waypoint)
            #     add_point = False
            # else:
            #     add_point = True

        # self.waypoints = smoothed_points

        del self.waypoints[-1]
        del self.waypoints[-1]
        del self.waypoints[-1]

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

                    absolute_truth = self.create.sim_get_position()

                    print("Absolute position - ", absolute_truth)

                    if abs(0.50 - absolute_truth[0]) <= .20 and abs(1.50 - absolute_truth[1]) <= .20:
                        print("Trigger abs position!")
                        self.create.drive_direct(0,0)
                        print("Movement should be stopped")
                        return

                    if abs(goal_x - self.odometry.x) <= .15 and abs(goal_y - self.odometry.y) <= .15:
                        self.create.drive_direct(0,0)
                        print("Movement should be stopped")
                        break

                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    # improved version 2: fuse with velocity controller
                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))


