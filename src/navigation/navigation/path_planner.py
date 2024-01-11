import goal_manager
import numpy as np 
from scipy.spatial import KDTree
from shapely.geometry import Point, LineString, Polygon


class PathPlanner():
    def __init__(self):
        self.target_pos = goal_manager.GoalManager()
        self.current_position = "init starting position from csv file"
        self.checkpoints = "read checkpoint locations from csv file"
        self.obstacles = "read obstacle locations"
    
    def get_next_checkpoint(self, 
                            current_position: tuple(float, float), 
                            target_pos: goal_manager.GoalManager) -> tuple(bool, tuple(float, float)):
        
        """
        checks if 
        """

        return True, (1, 0)
    

    def is_in_range(self, 
                    current_position: tuple(float, float), 
                    target_coordinate: tuple(float, float)) -> bool:
        """
        Recieved current_position and target coordinate
        Checks if current_position and target coordinate are within relative range
        returns boolean
        """

    def global_prm(self, ):
        
        def is_free(x: float, y: float, obstacles: list[tuple(float,float)]) -> bool:
            point = Point(x,y)
            return not any(point.within(obstacle) for obstacle in ob)

        return False


    

    