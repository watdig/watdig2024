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
        '''
        Sets a new goal.
        '''
        self.current_goal = (float_one, float_two)
        print(f"Current goal set: {self.current_goal}")

    def update_goal(self, float_one: float, float_two: float) -> None:
        """
        Updates the current goal.
        """
        if self.current_goal is not None:
            self.current_goal = (float_one, float_two)
        else:
            print("No current goal, please use set_goal method first")

    def is_goal_reached(self, current_position: tuple[float, float]) -> bool:
        """
        Checks if the goal has been reached.
        """
        if self.current_goal is None:
            print("No current goal set")
            return False

        if self.current_goal == current_position:
            print(f"Target {self.current_goal} reached")
            return True

        return False

    def get_current_goal(self) -> tuple[float, float]:
        '''
        Returns the current goal.
        '''
        if self.current_goal is None:
            print('No goal set')
        return self.current_goal
