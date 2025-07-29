"""
Module to define control behavior functions used in dynamics.py

Author:
    Aarush Banerjee
"""

import config as C
import numpy as np
import numpy.linalg as mm
import control as ctrl
import index
import vehiclemodel as vm
import thrust
import rotations as rot
from math import asin, acos, cos, sin, tan, atan2, pi, sqrt, hypot, copysign, inf, isclose

def generate_orientation_controller(V: vm.Vehicle, rho: float = 1, Q: list[list[float]] = None, R: list[list[float]] = None) -> list[list[float]]:
    """
    Generates and returns an optimal LQR gain K for the vehicle's 2D SISO orientation system
    
    Args:
        V (Vehicle): a vehicle object from which to extract physical quantities
        rho (float): a scalar value representing the weight of the input cost in the LQR controller
    Returns:
        list[list[float]]: an mxn matrix representing the LQR gains
    """

    # state dynamics matrix
    A = np.array(
        [
            #0  0'
            [0, 1                                        ], # 0'
            [0, -V.CD_thetadot * C.rho * V.length * V.dia]  # 0'' (linearizing about 0' ~= 1 rad/s)
        ]
    )

    # input dynamics matrix
    B = np.array(
        [
            #u
            [0                                              ], # 0'
            [-mm.norm(np.array(V.CoT) - np.array(V.CoG))/V.J]  # 0''
        ]
    )

    if mm.matrix_rank(ctrl.ctrb(A, B)) != np.shape(A)[0]:
        raise Exception("System is not controllable")

    # state cost weighting
    if Q is None or np.shape(Q) != (2,2): 
        Q = np.array(
            [
                [1/rho, 0       ],
                [0,     0.01/rho]
            ]
        )
    else:
        Q = np.array(Q)

    # input cost weighting
    if R is None or np.shape(R) != (1, 1):
        R = np.array(
            [
                [
                    rho
                ]
            ]
        )
    else:
        R = np.array(R)

    K = ctrl.lqr(A, B, Q, R)[0]

    V.K_orientation = K

    return K.tolist()

prev_time: float = 0
prev_theta_dot: float = 0
def update_controls(V: vm.Vehicle, time: float, X: float) -> list[float]:
    """
    Updates state machines and handles vehicle's control behavior. Returns commanded state for internal LQR loop.

    Args: 
        V (Vehicle): a vehicle instance
    """

    alt = X[index.Y]
    descent_rate = X[index.Y_DOT]
    alt_thresh = 30
    theta_comm = pi/2
    
    theta = X[index.THETA] % (2*pi)
    if theta > (theta_comm + pi): theta -= 2*pi
    if theta < (theta_comm - pi): theta += 2*pi

    if V.thrust_sources[0].get_ignition_time() is inf:
        V.thrust_sources[0].ignite(time) 
    
    if (V.thrust_sources[1].get_ignition_time() is inf) and (alt < alt_thresh) and (descent_rate < 0) and (time > 3):
        V.thrust_sources[1].ignite(time)

    theta_err = theta_comm - theta
    theta_dot_err = -X[index.THETA_DOT]

    U: list[float] = [0]

    # calculate perpendicular thrust component
    U[index.U_PERP] += (V.K_orientation @ np.array([[theta_err], [theta_dot_err]])).tolist()[0][0]

    # perpendicular drag disturbance feedforward term
    ydotbody = (rot.R_ib(X[index.THETA]) @ np.array([X[index.X_DOT], X[index.Y_DOT]]))[1]
    drag_ang_acc_est = (-ydotbody/abs(ydotbody))*((1/2) * C.rho * (V.length * V.dia) * V.CD_flat * mm.norm(np.array(V.CoP) - np.array(V.CoG))/V.J) * ( # W hat airspeed
        ydotbody # body-frame y dot
    )**2 # W hat
    U[index.U_PERP] -= -(drag_ang_acc_est/(mm.norm(np.array(V.CoT) - np.array(V.CoG))/V.J)) 

    # bias calculation and feedforward
    
    # theta_dot_hat = prev_theta_dot + (drag_ang_acc_est) * (time - prev_time) 

    # prev_time = time
    # prev_u = U[0]
    
    return U
    

def calc_thrust_vector(V: vm.Vehicle, thruster: thrust.ThrustSource, time: float, X: list[float], U: list[float]) -> list[float]:

    if isinstance(thruster, thrust.SolidMotor):
        # handling solid motors

        perp = U[index.U_PERP] if abs(U[index.U_PERP]) < thruster.thrust_mag(time) else copysign(thruster.thrust_mag(time), U[index.U_PERP]) # with limits
        
        return [
                0 if (abs(perp) >= thruster.thrust_mag(time)) else sqrt((thruster.thrust_mag(time)**2) - (perp**2)), # parallel thrust
                perp # perpendicular thrust with limits
                ]

    else: 
        raise Exception("No calc_thrust_vector() behavior was defined for this ThrustSource subclass")