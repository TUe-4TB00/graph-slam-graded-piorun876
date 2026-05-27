
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X # type: ignore

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # type: ignore # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # type: ignore # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # type: ignore # (bearing, range)

def add_pose(graph, initial_estimate):
    # TODO: Add the odometry factor between X(3) and X(4) to the graph (BetweenFactorPose2)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), gtsam.Pose2(2**0.5, 2**0.5, math.pi/2), ODOMETRY_NOISE)) # type: ignore

    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph

    # initial_estimate.insert(X(1), gtsam.Pose2(-0.25, 0.20, 0.15))
    # initial_estimate.insert(X(2), gtsam.Pose2(2.30, 0.10, -0.20))
    # initial_estimate.insert(X(3), gtsam.Pose2(4.10, 0.10, 0.10))
    prevPos = np.array([4, 0, 0])
    movement = np.array([2 * math.cos(math.pi/4), 2 * math.sin(math.pi/4), math.pi/2])
    nextPos = prevPos + movement
    # print(prevPos)
    # print(movement)
    # print(nextPos)

    initial_estimate.insert(X(4), gtsam.Pose2(nextPos[0], nextPos[1], nextPos[2])) # type: ignore
    
    return graph, initial_estimate