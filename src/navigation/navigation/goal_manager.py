import math

"""
Manages goals for the navigation package.
"""


class GoalManager:
    """
    Initialization with no goal.
    """

    def __init__(self):
        # Initialize with no goal.
        self.current_goal = None

    def set_goal(self, float_one: float, float_two: float) -> None:
        """sets coordinate goal

        Args:
            float_one (float): _description_
            float_two (float): _description_
        """
        self.current_goal = (float_one, float_two)
        print(f"Current goal set: {self.current_goal}")

    def update_goal(self, coordinate: tuple[float, float]) -> None:
        """updates coordinate goal

        Args:
            coordinate (tuple[float, float]): _description_
        """
        if self.current_goal is not None:
            self.current_goal = coordinate
        else:
            print("No current goal, please use set_goal method first")

    def is_goal_reached(self, current_position: tuple[float, float]) -> bool:
        """checks whether current position is within tolerance of goal

        Args:
            current_position (tuple[float, float]): _description_

        Returns:
            bool: _description_
        """
        if self.current_goal is None:
            print("No current goal set")
            return False

        radius = 1.1

        distance_squared = (self.current_goal[0] - current_position[0]) ** 2 + (
            self.current_goal[1] - current_position[1]
        ) ** 2

        distance = math.sqrt(distance_squared)

        return distance < radius

    def get_current_goal(self) -> tuple[float, float]:
        """returns the current goal

        Returns:
            tuple[float, float]: _description_
        """
        if self.current_goal is None:
            print("No goal set")
        return self.current_goal
