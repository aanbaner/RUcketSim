"""
Module that contains a vehicle class used for maintaining, tracking, and passing the state vector of a vehicle to other modules.

Author:
    Aarush Banerjee
"""

import config as C
import thrust
from math import pi

class Vehicle:
    def __init__(self,
                 m: float = C.m, 
                 J: float = C.J, 
                 CoG: list[float] = C.CoG, 
                 CoP: list[float] = C.CoP, 
                 CoT: list[float] = C.CoT, 
                 base: list[float] = C.base,
                 length: float = C.length,
                 dia: float = C.dia,
                 max_gimbal_angle: float = C.max_gimbal_angle,
                 max_gimbal_angle_rate: float = C.max_gimbal_angle_rate,
                 max_thrust: float = C.max_thrust,
                 min_thrust: float = C.min_thrust,
                 max_thrust_rate: float = C.max_thrust_rate,
                 CD_0: float = C.CD_0,
                 CD_thetadot: float = C.CD_thetadot,
                 CD_flat: float = C.CD_flat,
                 CL_alpha: float = C.CL_alpha,
                 K_orientation: list[list[float]] = None,
                 rho: float = 1.,
                 ): 
        """
        Creates a Vehicle Object with initial conditions for state and control vectors
        """
        self.thrust_sources: list[thrust.ThrustSource] = []

        self.time: float = 0 # domain

        # physical constants
        self.m: float = m
        self.J: float = J
        self.CoG: list[float] = CoG
        self.CoT: list[float] = CoT
        self.CoP: list[float] = CoP
        self.base: list[float] = base
        self.length: float = length
        self.dia: float = dia
        self.max_gimbal_angle: float = max_gimbal_angle
        self.max_gimbal_angle_rate: float = max_gimbal_angle_rate
        self.max_thrust: float = max_thrust
        self.min_thrust: float = min_thrust
        self.max_thrust_rate: float = max_thrust_rate
        self.CD_0: float = CD_0
        self.CD_thetadot: float = CD_thetadot
        self.CD_flat: float = CD_flat
        self.CL_alpha: float = CL_alpha

        # utils
        if K_orientation is None:
            from controller import generate_orientation_controller
            self.K_orientation: list[list[float]] = generate_orientation_controller(self,rho=rho)
        else:
            self.K_orientation: list[list[float]] = K_orientation
    
        self.state = 'INIT'

    def SolidMotor(self, csv_filename: str, CoT: list[float] = C.CoT, qty: int = 1, skiprows: int = 3, interp_order: int = C.THRUST_INTERP_ORDER, characteristic_thrust: float = None):
        """
        Creates a SolidMotor object attached to the vehicle, and appends to the thrust_sources list
        """
        new_motor = thrust.SolidMotor(csv_filename=csv_filename, CoT=CoT, qty=qty, skiprows=skiprows, interp_order=interp_order, characteristic_thrust=characteristic_thrust)
        self.thrust_sources.append(new_motor)
        return new_motor

    def update(self, t: float, y: list[float]) -> list[float]:
        """
        Update wrapper function of the vehicle according to the dynamics described in dynamics.py 
        and controller.py. Function signature designed for use in integrators such as scipy.integrate.RK45
        
        NOTE: this function locally imports dynamics.py

        Args:
            t (float): time at which to evaluate the dynamics of the vehicle
            y (list[float]): state at which to evaluate the dynamics of the vehicle 

        Returns:
            list[float]: nx1 array corresponding to the vehicle's state

        """

        from dynamics import update_Xdot

        return update_Xdot(self, t, y)







