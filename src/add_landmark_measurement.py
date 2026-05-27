import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X # type: ignore

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # type: ignore # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # type: ignore # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # type: ignore # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 

    posL2 = np.array([4, 2])
    posX4 = np.array([4+2**0.5, 2**0.5])

    dx = posX4[0] - posL2[0]
    dy = posX4[1] - posL2[1]

    d = (dx**2 + dy**2)**0.5

    rotation = math.asin(dx/d)*180/math.pi
    distance = d
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2.fromDegrees(rotation), distance, MEASUREMENT_NOISE)) # type: ignore
    return graph