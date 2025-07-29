"""
2D Rotation functions

Author:
    Aarush Banerjee
"""

from math import sin, cos
import numpy as np

def R_ib(theta: float) -> list[list[float]]:
    """
    Returns the 2-D inertial to body rotation matrix based on a counter clockwise rotation theta

    Args:
        theta (float): angle by which to rotate
    """
    return [[ cos(theta), sin(theta)],
            [-sin(theta), cos(theta)]]

def R_bi(theta: float) -> list[list[float]]:
    """
    Returns the 2-D body to inertial rotation matrix based on a clockwise rotation theta

    Args:
        theta (float): angle by which to rotate
    """
    return [[cos(theta), -sin(theta)],
            [sin(theta),  cos(theta)]]

def rot_ib(theta: list[float], vertices: list[list[float]]) -> list[list[float]]:
    c = np.cos(theta)
    s = np.sin(theta)

    vert = np.transpose(vertices)

    vertprime = np.array(
        [   
            c * vert[0] + s * vert[1],
            c * vert[1] - s * vert[0]
        ]
    )

    return vertprime.transpose().tolist()

def rot_bi(theta: list[float], vertices: list[list[float]]) -> list[list[float]]:
    c = np.cos(theta)
    s = np.sin(theta)

    vert = np.transpose(vertices)

    vertprime = np.array(
        [   
            c * vert[0] - s * vert[1],
            c * vert[1] + s * vert[0]
        ]
    )

    return vertprime.transpose().tolist()