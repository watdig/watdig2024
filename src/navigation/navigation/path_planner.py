import goal_manager
import numpy as np
import networkx as nx
import math
from scipy.spatial import KDTree
from shapely.geometry import Point, LineString, Polygon


class PathPlanner:
    def __init__(self):
        self.target_pos = goal_manager.GoalManager()
        self.environment = {}
        self.checkpoints = []
        self.obstacles = []
        self.targets = []
        self.index = 0
        self.maxlen = 0
        self.angle = 0
        self.distance = 0

    def get_next_checkpoint(
        self,
        current_position: tuple(float, float),
    ):
        """
        if checkpoint is in range, identifies next checkpoint to travel to based on PRM
        """
        reached_goal = self.is_in_range(current_position)
        if reached_goal:
            if self.index < self.maxlen:
                self.target_pos.update_goal(self.targets[self.index]) # identify next goal based on prm
                self.index+=1
                self.angle = self.calculate_angle_between_points(current_position, self.target_pos.current_goal)
                self.distance = self.calculate_distance_between_points(current_position, self.target_pos.current_goal)
        return True
    
    def recalculate_route(self,
                            current_position: tuple(float, float)):
        self.angle = self.calculate_angle_between_points(current_position, self.target_pos.current_goal)
        self.distance = self.calculate_distance_between_points(current_position, self.target_pos.current_goal)
        return True

    def is_in_range(self, current_position: tuple(float, float)) -> bool:
        """
        Checks if current_position and target coordinate are within relative range
        returns boolean
        """
        return self.target_pos.is_goal_reached(current_position=current_position)

    def calculate_distance_between_points(
        self, coordinate1: tuple(float, float), coordinate2: tuple(float, float)
    ) -> float:
        """
        calculates distance between two points
        """
        distance = math.sqrt(
            (coordinate1[0] - coordinate2[0]) ** 2
            + (coordinate1[1] - coordinate2[1]) ** 2
        )

        return distance

    def calculate_angle_between_points(
        self, position: tuple(float, float), target: tuple(float, float)
    ) -> float:
        """
        calculates the angle between any two points
        """
        dx = target[0] - position[0]
        dy = target[1] - position[1]

        angle_radians = math.atan2(dy, dx)
        angle_degrees = math.degrees(angle_radians)

        # modify by 90 degrees right or left/add absolute value depending on 0 axis

        return angle_degrees

    def global_prm(
        self,
    ):
        """
        runs in the init part of the node
        """
        start = self.environment["origin"]
        finish = self.environment["finish"]
        
        # Todo; VERIFY ARRAYS OF INFORMATION FROM TOPIC WORK
        
        
        # Parameters
        NUM_SAMPLES = 400
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

        self.targets = path
        self.maxlen = len(path)
        
        
        return True
