from scipy.optimize import minimize
import numpy as np

def location_solver(point1, point2, distances):
    # Adjusted objective function to minimize
    def objective_func(X):
        x, y = X
        return sum([((x - point[0])**2 + (y - point[1])**2 - d**2)**2 for point, d in zip([point1, point2], distances)])


    # Compute the midpoint of the line segment connecting point1 and point2
    midpoint = np.array([(point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2])
    
    x0 = np.array([20,20])
    
    # Perform the minimization with adjusted objective function
    result = minimize(objective_func, x0, method='L-BFGS-B')
    
    if result.success:
        # Ensuring the solution has positive coordinates
        if result.x[0] >= 0 and result.x[1] >= 0:
            return result.x
        else:
            return "Solution has non-positive coordinates."
    else:
        return "Optimization failed."


if __name__ == '__main__':
    # Attempting the solution again with the adjustments
    res = location_solver((0,0), (15,0), [5, 10])
    print(res)
