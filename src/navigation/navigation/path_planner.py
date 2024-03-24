import logging
from navigation.goal_manager import GoalManager
import numpy as np
import networkx as nx
import math
from scipy.spatial import KDTree
from shapely.geometry import Point, LineString, Polygon
import matplotlib.pyplot as plt

class PathPlanner:
    def __init__(self):
        self.target_pos = GoalManager()
        self.environment = {}
        self.checkpoints = []
        self.obstacles = []
        self.targets = []
        self.index = 0
        self.num_nodes = 0
        self.angle = 0
        self.distance = 0

    def get_next_checkpoint(
        self,
        current_position: list[float, float],
    ):
        """
        if checkpoint is in range, identifies next checkpoint to travel to based on PRM
        """
        reached_goal = self.is_in_range(current_position)
        if reached_goal:
            if self.index < self.num_nodes:
                self.target_pos.update_goal(self.targets[self.index]) # identify next goal based on prm
                self.index+=1
                self.angle = self.calculate_angle_for_gyroscope(current_position, self.target_pos.current_goal)
                self.distance = self.calculate_distance_between_points(current_position, self.target_pos.current_goal)
        return True
    
    def recalculate_route(self,
                            current_position: list[float, float]):
        self.angle = self.calculate_angle_for_gyroscope(current_position, self.target_pos.current_goal)
        self.distance = self.calculate_distance_between_points(current_position, self.target_pos.current_goal)
        return True

    def is_in_range(self, current_position: list[float, float]) -> bool:
        """
        Checks if current_position and target coordinate are within relative range
        returns boolean
        """
        return self.target_pos.is_goal_reached(current_position=current_position)

    def calculate_distance_between_points(
        self, coordinate1: list[float, float], coordinate2: list[float, float]
    ) -> float:
        """
        calculates distance between two points
        """
        distance = math.sqrt(
            (coordinate1[0] - coordinate2[0]) ** 2
            + (coordinate1[1] - coordinate2[1]) ** 2
        )

        return distance

    def calculate_angle_for_gyroscope(self, position: list[float], target: list[float]) -> float:
        """
        Calculates the angle between two points (position and target),
        adjusted for a gyroscope where 90 degrees is the positive x-axis.
        The function ensures that angles are given in a range suitable for the gyroscope,
        increasing counterclockwise with 0 degrees at the unit circle's 90 degrees.
        """
        dx = target[0] - position[0]
        dy = target[1] - position[1]

        angle_radians = math.atan2(dy, dx)
        angle_degrees = math.degrees(angle_radians)

        # Adjust the angle so that 0 degrees is at the gyroscope's 90 degrees
        # and correct the direction to match the gyroscope's orientation
        if angle_degrees > 90:
            angle_degrees = 360 - (angle_degrees - 90)
        else:
            angle_degrees = 90 - angle_degrees

        return angle_degrees

    def global_prm(self):
        """
        Initializes the PRM.
        """
        logger = logging.getLogger()
        start = Point(self.environment["origin"])
        finish = Point(self.environment["finish"])
        

        print(len(self.obstacles))
        print(len(self.checkpoints))

        for checkpoint in self.checkpoints:
            checkpoint = Point(checkpoint)
            logger.info(checkpoint)


        # Parameters
        NUM_SAMPLES = 50
        NEIGHBOR_RADIUS = 10

        # Define the boundary of the environment (example values)
        boundary = Polygon([self.environment["corner01"], self.environment["corner02"], self.environment["corner03"], self.environment["corner04"]])

        # Function to check if a point is in the free space
        def is_free(x, y):
            point = Point(x, y)
            return not any(
                point.within(obstacle) for obstacle in self.obstacles
            ) and point.within(boundary)

        # Function to check if a path between two points is free of obstacles
        def is_path_free(p1, p2):
            line = LineString([p1, p2])
            return not any(line.intersects(obstacle) for obstacle in self.obstacles)

        # Sample points
        samples = []
        while len(samples) < NUM_SAMPLES:
            x, y = np.random.uniform(0, 30), np.random.uniform(0, 30)
            if is_free(x, y):
                samples.append((x, y))

        # Ensure the start, checkpoints, and finish are in the samples
        important_points = [start] + self.checkpoints + [finish]
        for point in important_points:
            samples.append((point.x, point.y))

        # Create a KD-tree for efficient nearest neighbor search
        tree = KDTree(samples)

        # Build the graph
        graph = nx.Graph()
        for sample in samples:
            neighbors = tree.query_ball_point(sample, NEIGHBOR_RADIUS)
            for neighbor_idx in neighbors:
                neighbor = samples[neighbor_idx]
                if is_path_free(sample, neighbor):
                    graph.add_edge(
                        sample,
                        neighbor,
                        weight=np.linalg.norm(np.array(sample) - np.array(neighbor)),
                    )

        # Find the shortest path through the checkpoints
        path = []
        try:
            path.append(tuple(start.coords)[0])
            for checkpoint in self.checkpoints:
                path_segment = nx.shortest_path(graph, path[-1], tuple(checkpoint.coords)[0], weight="weight")
                path.extend(path_segment[1:])  # Exclude the first point to avoid duplication
            path.extend(nx.shortest_path(graph, path[-1], tuple(finish.coords)[0], weight="weight")[1:])
        except nx.NetworkXNoPath:
            print("No path could be found.")
            path = []  # Clear the path if no complete path could be found

        # Setting targets and maxlen array
        self.targets = path
        self.num_nodes = len(path)
        
        self.target_pos.set_goal(path[0])
        logger.info(self.target_pos.current_goal)
        
        # Logging information
        logger.info('Number of Nodes: %d', self.num_nodes)
        count = 1
        for target in self.targets:
            logger.info('Node Number %d: %s', count, target)
            count += 1

        return True
