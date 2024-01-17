import goal_manager
import numpy as np
import networkx as nx
import math
from scipy.spatial import KDTree
from shapely.geometry import Point, LineString, Polygon


class PathPlanner:
    def __init__(self):
        self.target_pos = goal_manager.GoalManager()
        self.current_position = "init starting position from csv file"
        self.checkpoints = "read checkpoint locations from csv file"
        self.obstacles = "read obstacle locations"
        self.angle = 0

    def get_next_checkpoint(
        self,
        current_position: tuple(float, float),
        target_pos: goal_manager.GoalManager,
    ) -> tuple(bool, tuple(float, float)):
        """
        checks if checkpoint is in range, identifies next checkpoint to travel to based on PRM
        """
        reached_goal = self.is_in_range(current_position=self.current_position)
        if reached_goal:
            new_goal = (0, 1)  # identify next goal based on prm
            target_pos.update_goal(new_goal)

            # calculate angle of rotation
            # get distance to travel

            return True, (1, 0)

    def is_in_range(self, current_position: tuple(float, float)) -> bool:
        """
        Recieved current_position and goal manager
        Checks if current_position and target coordinate are within relative range
        returns boolean
        """
        return self.target_pos.is_goal_reached(current_position=current_position)

    def update_current_position(self, position: tuple(float, float)) -> None:
        self.current_position = position

    def calculate_distance_between_points(
        self, coordinate1: tuple[float, float], coordinate2: tuple[float, float]
    ) -> float:
        """
        calculates distance between two points

        Args:
            coordinate1 (tuple[float, float]): _description_
            coordinate2 (tuple[float, float]): _description_

        Returns:
            float: _description_
        """
        distance = math.sqrt(
            (coordinate1[0] - coordinate2[0]) ** 2
            + (coordinate1[1] - coordinate2[1]) ** 2
        )

        return distance

    def calculate_angle_between_points(
        self, position: tuple[float, float], target: tuple[float, float]
    ) -> float:
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
        initial PRM, could be run as an action client when the program is started, could also make it a separate node
        """

        # Parameters
        NUM_SAMPLES = 400
        NEIGHBOR_RADIUS = 10

        # Obstacles with a bounding radius, represented as buffered points
        obstacles = [
            Point(5.4, 5.34).buffer(0.8),  # garyTheSnail
            Point(5, 15).buffer(1),  # theStrip1
            Point(8, 15).buffer(1),  # theStrip2
            Point(8, 15).buffer(
                1
            ),  # theStrip3 (assuming a typo in the image, adjusted for uniqueness)
            Point(8, 15).buffer(
                1
            ),  # theStrip4 (assuming a typo in the image, adjusted for uniqueness)
            Point(8, 6).buffer(1),  # theStrip5
            Point(8, 6).buffer(
                1
            ),  # theStrip6 (assuming a typo in the image, adjusted for uniqueness)
            Point(-8.654, 20.6).buffer(0.5),  # tunnel1
            Point(-7.89, 19).buffer(0.5),  # tunnel2
            Point(-8.675, 19.6).buffer(0.5),  # tunnel3
            # Add more obstacles if needed
        ]
        checkpoints = [
            Point(4, -0.5),  # gate01
            Point(4, 1.5),  # gate02
            Point(4.8, 3.8),  # gate03 (assumed coordinate, adjust as necessary)
            Point(5.6, 3.8),  # finishRamp
            Point(10.85, 24.96),
            # Add more checkpoints if needed
        ]
        start = Point(0, 0)  # Example start point
        finish = Point(23.126, 28.968)  # Example finish point

        # Define the boundary of the environment (example values)
        boundary = Polygon([(0, 0), (0, 30), (30, 30), (30, 0)])

        # Function to check if a point is in the free space
        def is_free(x, y):
            point = Point(x, y)
            return not any(
                point.within(obstacle) for obstacle in obstacles
            ) and point.within(boundary)

        # Function to check if a path between two points is free of obstacles
        def is_path_free(p1, p2):
            line = LineString([p1, p2])
            return not any(line.intersects(obstacle) for obstacle in obstacles)

        # Sample points
        samples = []
        while len(samples) < NUM_SAMPLES:
            x, y = np.random.uniform(0, 30), np.random.uniform(0, 30)
            if is_free(x, y):
                samples.append((x, y))

        # Ensure the start, checkpoints, and finish are in the samples
        important_points = [start] + checkpoints + [finish]
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
        try:
            path = [tuple(start.coords)[0]]
            for checkpoint in checkpoints:
                path_segment = nx.shortest_path(
                    graph, path[-1], tuple(checkpoint.coords)[0], weight="weight"
                )
                path.extend(
                    path_segment[1:]
                )  # Exclude the first point to avoid duplication
            path.extend(
                nx.shortest_path(
                    graph, path[-1], tuple(finish.coords)[0], weight="weight"
                )[1:]
            )
        except nx.NetworkXNoPath as e:
            print(f"No path could be found: {e}")
            path = []

        return False
