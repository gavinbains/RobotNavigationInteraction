"""
Code for PA2
Use "run.py [--sim] pa2" to execute
"""
import math
import numpy as np
import random

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        self.arm = factory.create_kuka_lbr4p()
        self.time = factory.create_time_helper()
        self.l1 = 0.4
        self.l2 = 0.4
        self.x1 = 0
        self.y1 = 0.7
        self.x2 = 0
        self.y2 = 0
        self.x = 0
        self.y = 1
        self.theta1 = 0
        self.theta2 = 0
        self.r = math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))

    def calc_ik(self, x, y):
        self.r = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

        beta_numerator = math.pow(self.l1, 2) + math.pow(self.r, 2) - math.pow(self.l2, 2)
        beta_denom = 2 * self.l1 * self.r
        beta = math.acos(beta_numerator/beta_denom)

        alpha_numerator = math.pow(self.l1, 2) + math.pow(self.l2, 2) - math.pow(self.r, 2)
        alpha_denom = 2 * self.l1 * self.l2
        alpha = math.acos(alpha_numerator/alpha_denom)

        subtract_beta = math.atan2(x, y) - beta
        add_beta = math.atan2(x, y) + beta
        subtract_alpha = math.pi - alpha
        add_alpha = math.pi + alpha

        if abs(math.degrees(subtract_beta)) > 135:
            self.theta1 = add_beta
            self.theta2 = add_alpha
        else:
            self.theta1 = subtract_beta
            self.theta2 = subtract_alpha

        return math.radians(90) - self.theta1, math.radians(90) - self.theta2

    def calc_location(self):
        self.x1 = self.l1 * math.cos(self.theta1)
        self.y1 = self.l1 * math.sin(self.theta1)
        self.x2 = self.l2 * math.cos(self.theta1 + self.theta2)
        self.y2 = self.l2 * math.sin(self.theta1 + self.theta2)
        self.x = self.x1 + self.x2
        self.y = self.y1 + self.y2

        return self.x, (self.y + 0.37)

    def print_forward_kinematics(self, joint, theta):
        self.arm.go_to(joint, theta)
        if joint is 1:
            self.theta1 = math.radians(90) - theta
        elif joint is 3:
            self.theta2 = math.radians(90) - theta
        coord = self.calc_location()
        print("Go to", joint, math.degrees(theta), "deg , FK: [", -coord[0], coord[1], "0]")

    def print_inverse_kinematics(self, x, y):
        angle = self.calc_ik(x, y - 0.37)
        self.arm.enable_painting()
        self.arm.go_to(1, self.theta1)
        self.arm.go_to(3, self.theta2)
        self.time.sleep(1)
        print("Go to [", x, ",", y, "], IK: [", math.degrees(angle[0]), ",", math.degrees(angle[1]), "]")

    @staticmethod
    def get_points(p1, p2, parts):
        return zip(np.linspace(p1[0], p2[0], parts + 1), np.linspace(p1[1], p2[1], parts + 1))

    def go_to_cup_plane(self):
        self.arm.open_gripper()
        for x in range(18):
            self.arm.go_to(1, math.radians(x*5))
            self.time.sleep(1)

    def grab_cup(self):
        self.arm.go_to(5, math.radians(20))
        self.arm.close_gripper()
        self.time.sleep(5)
        self.arm.go_to(5, math.radians(0))
        self.arm.go_to(1, math.radians(80))

    def prepare_cup(self):
        for x in range(36):
            self.arm.go_to(6, math.radians(-(x*5)))
            self.time.sleep(1)
        for x in range(18):
            self.arm.go_to(5, math.radians(-(x*5)))
            self.time.sleep(1)
        self.arm.go_to(1, math.radians(0))

    def run(self):
        # print("joint 0 - Twists L1")
        # print("joint 1 - Moves L1")
        # print("joint 2 - Twist base of L2")
        # print("joint 3 - Moves L2")
        # print("joint 4 - Twists base of End Effector")
        # print("joint 5 - Moves End Effector")
        # print("joint 6 - Twists End Effector")
        # self.go_to_cup_plane()
        # self.grab_cup()
        # self.time.sleep(2)
        # self.prepare_cup()

        # second is Z
        # first is X

        # get ground truth of cup for x coord, add .08 to it, and then thats the first number
        # set z constant at 0.2 with a difference of 0.04 off of the starting position

        self.time.sleep(2)
        self.arm.open_gripper()
        self.time.sleep(2)
        self.arm.go_to(5, math.radians(-30))
        self.print_inverse_kinematics(0.75, 0.2)
        self.time.sleep(2)
        self.arm.close_gripper()
        self.time.sleep(4)

        points = list(self.get_points((0.75, 0.2), (0, 1), 100))
        for coord in points:
            self.print_inverse_kinematics(coord[0], coord[1])
        self.time.sleep(2)

        for x in range(36):
            self.arm.go_to(6, math.radians(-(x * 5)))
            self.time.sleep(1)

        points = list(self.get_points((0, 1), (-0.5, 0.65), 100))
        for coord in points:
            self.print_inverse_kinematics(coord[0], coord[1])

        # self.print_inverse_kinematics(-0.5, 0.6)
        self.time.sleep(2)
        for x in range(36):
            self.arm.go_to(5, math.radians(-(30+x * 2.5)))
            self.time.sleep(1)

        self.arm.open_gripper()
        while True:
            self.time.sleep(1)
        pass
