import math
from collections import defaultdict


class RRT:
    def __init__(self, start_coords, end_coords):
        self.start = start_coords
        self.end = end_coords
        self.nodes = []
        self.nodes.append(start_coords)
        self.edges = []
        self.map = defaultdict(list)
        self.weights = {}
        self.parents = {}

    def add_vertex(self, vertex):
        self.nodes.append(vertex)

    def add_edge(self, v1, v2):
        self.edges.append((v1, v2))
        self.map[v2].append(v1)
        self.weights[(v1, v2)] = self.dist(v1, v2)
        self.parents[v2] = v1

    def print_shortest(self):
        shortest_path = []
        current_node = self.end
        while current_node != self.start:
            print("current node:", current_node)
            shortest_path.append(self.parents[current_node])
            current_node = self.parents[current_node]
        return shortest_path

    @staticmethod
    def dist(p1, p2):
        return math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))
