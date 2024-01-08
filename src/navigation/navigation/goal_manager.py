class GoalManager():
    def __init__(self):
        #initialize with no goal
        self.current_goal = None


    def set_goal(self, x: float, y:float) -> None:
        '''
        sets a new goal
        '''
        self.current_goal = (x, y)
        print(f"current goal set:  {self.current_goal}")

    def update_goal(self, x:float, y:float) -> None:
        """
        updates current goal
        """
        if self.current_goal is not None:
            self.current_goal = (x, y)
        else:
            print("No current goal, please use set_goal method first")

    def is_goal_reached(self, current_position: tuple[float,float]) -> bool:
        """
        checks if goal has been reached
        """   
        #todo: add tolerances  
        if self.current_goal is None:
            print("no current goal set")
            return False
        
        if self.current_goal == current_position:
            print(f"target {self.current_goal} reached")
            return True
        
        else:
            return False
    
    def get_current_goal(self) -> tuple[float, float]:
        '''
        returns current goal 
        '''
        if self.current_goal is None:
            print('no goal set')
        return self.current_goal            

        
        
