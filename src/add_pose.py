
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X # type: ignore

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # type: ignore # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # type: ignore # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # type: ignore # (bearing, range)

def add_pose(graph, initial_estimate):
    # TODO: Add the odometry factor between X(4) and X(5) to the graph (BetweenFactorPose2)

    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph
    
    return graph, initial_estimate