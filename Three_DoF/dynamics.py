"""
Module managing state-space dynamics equaitons for simulation.
This module provides functionality for gravity and airspeed forces.

Author:
    Aarush Banerjee
"""

import rotations as rot
import config as C
import index
import vehiclemodel as vm
import controller as ctrlr
import numpy as np
import numpy.linalg as mm
from math import sin, asin, cos, acos, tan, atan2, hypot, pi, nan, isclose, degrees

def update_Xdot(V: vm.Vehicle, time: float, X: list[float]) -> list[float]:
    dot = np.zeros_like(X)

    # update controller
    U = ctrlr.update_controls(V, time, X)

    # get all our forces
    forces, levers, moments = calc_forces_levers_moments(V, time, X, U)

    # convert to linear and angular acceleration
    body_lin_accel, ang_accel = calc_body_2nd_deriv_2D(forces, levers, moments, V)

    # convert linear acceleration to inertial frame
    lin_accel = rot.R_bi(X[index.THETA]) @ np.array(body_lin_accel)

    # assigning dot vector
    dot[index.X:index.THETA + 1] = X[index.X_DOT:index.THETA_DOT + 1] # trivialities
    dot[index.X_DOT:index.Y_DOT + 1] = lin_accel.tolist() # linear accelerations
    dot[index.THETA_DOT] = ang_accel # angular accelerations

    return dot

def calc_forces_levers_moments(V: vm.Vehicle, time: float, X: list[float], U: list[float]) -> tuple[list[list[float]], list[list[float]], list[float]]:
    forces: list[list[float]] = []
    levers: list[list[float]] = []
    moments: list[float] = []

    # applying drag to center of pressure
    FD, MD = calc_aerodynamics(V, X)
    forces.append(FD)
    levers.append(V.CoP)
    moments.append(MD)

    # applying thrust vectors of each thrust source to their respective centers of thrust
    for thrust_source in V.thrust_sources:
        forces.append(ctrlr.calc_thrust_vector(V, thrust_source, time, X, U))
        levers.append(thrust_source.center_of_thrust())

    # applying gravity to center of mass/gravity
    forces.append(rot.R_ib(X[index.THETA]) @ np.array([0., - V.m * C.g]))
    levers.append(V.CoG)

    return forces, levers, moments

def calc_body_2nd_deriv_2D(forces: list[list[float]], levers: list[list[float]], moments: list[float], V: vm.Vehicle) -> tuple[list[float], float]:
    """
    Takes in a set of forces and associated lever arms and returns combined linear and angular acceleration vectors and values. 
    Forces and lever arms are associated with one another based on index (e.g. forces[3] corresponds to levers[3], etc.).
    Arguments must have equal amounts of forces and levers, and forces and levers must both only be 2D.

    Args:
        forces (list[list[float]]): a list of 2-vectors representing forces acting on the vehicle in the body-frame.
        levers (list[list[float]]): a list of 2-vectors representing lever arms corresponding to forces of the same index.
        V (Vehicle): a vehicle object from which to extract physical quantities

    Returns:
        tuple[list[float], float]: a tuple containing the linear acceleration vector and angular acceleration value.
    """

    # ensuring forces and levers are homogenous with one another
    try:
        if np.shape(forces) != np.shape(levers):
            raise Exception("Not every force or lever arm is associated with a counterpart, or forces and lever arms have different vector dimensionalities")
    except:
        raise Exception("Inhomogenous forces or lever arm vector dimensionalities")
    # testing the dimensionality
    if (np.shape(forces)[1] == 2):
        a = np.array([0., 0.]) # initalize a 2-vector for linear acceleration vector
        alpha = np.array(0.) # initialize an angular acceleration value
    else:   
        raise Exception("Vector dimensionality must be 2")

    # iterate through forces
    for i in range(len(forces)):
        a += forces[i] # sum force application to CoG
        alpha += np.cross(levers[i], forces[i]) # sum cross product to lever arm to moments

    for i in range(len(moments)):
        alpha += moments[i]

    # use inertial quantities to transform to positional derivatives
    a /= V.m
    alpha /= V.J

    return (a.tolist()), alpha.tolist()

def calc_coefficients(V: vm.Vehicle, alpha: float) -> tuple[float, float, float]:
    # surface area calculations
    S_circ = (pi * ((V.dia/2)**2))
    S_side = (V.length * V.dia)

    # drag model

        # Linear drag force
    SCD = (V.CD_0 
          * S_circ * abs(cos(alpha))) + (V.CD_flat 
                                         * S_side * abs(sin(alpha))) # area of a rectangular side profile cross axis of attack
    
        # Drag moment
    SCDm = (V.CD_thetadot 
           * S_side) # area of rectangular side profile

    # lift model

    SCL = (
            V.CL_alpha
            * (V.length * V.dia) * sin(2 * alpha) # area of rectangular side profile times flat plate angle model
            )
    
    return SCD, SCL, SCDm

def calc_aerodynamics(V: vm.Vehicle, X: list[float]) -> tuple[list[float], float]:
    """
    Calculates the drag and lift forces on a Vehicle acting on the vehicle's CoP.

    Args:
        V (Vehicle): a vehicle instance containing state and parameter values.
    
    Returns:
        list[float]: 2-vector representing drag and lift forces acting at V's CoP.
    """

    Va: np.ndarray = rot.R_ib(X[index.THETA]) @ np.array([X[index.X_DOT], X[index.Y_DOT]])
    
    alpha = atan2(Va[1], Va[0]) # angle of attack
    mag_Va: float = mm.norm(Va)

    SCD, SCL, SCDm = calc_coefficients(V, alpha)
    
    if (isclose(mag_Va, 0.0)):
        FD = np.array([0., 0.])
        FL = np.array([0., 0.])
    else:
        FD = ((1/2) * C.rho * SCD * (mag_Va**2)) * (
            -Va / mag_Va) # parallel to airspeed
        FL = ((1/2) * C.rho * SCL * (mag_Va**2)) * (
            (rot.R_ib(pi/2) @ -Va) / mag_Va) # perpendicular to airspeed

    if (isclose(abs(X[index.THETA_DOT]), 0.)):
        MD = 0.
    else:
        MD = ((1/2) * C.rho * SCDm * (X[index.THETA_DOT]**2)) * (- X[index.THETA_DOT] / abs(X[index.THETA_DOT]))

    return (FD + FL).tolist(), MD


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from scipy.integrate import solve_ivp
    from draw import draw_traj, draw_rocket, draw_forces, draw_states

    print("Dynamics module test harness beginning")

    # testing that with no thrust, the vehicle's 
    X0=[0,0,0,0,0,0]
    V = vm.Vehicle()

    # 0 magnitude solidmotors to ensure that the controller doesnt reference nonexisent items
    V.SolidMotor('ThrustCurves/Estes_E12.csv', qty=0)
    V.SolidMotor('ThrustCurves/Estes_E12.csv', qty=0)

    # integrate
    result = solve_ivp(V.update, [0, 30], X0, 'RK45', np.linspace(0, 30, 250))

    # data post processing
        # get forces and levers for drawing
    F = np.zeros((len(result.t), 2 + len(V.thrust_sources), 2))
    L = np.zeros((len(result.t), 2 + len(V.thrust_sources), 2))
    for i in (range(len(result.t))):
        forces, levers, moments = calc_forces_levers_moments(V, result.t[i], result.y[:,i], ctrlr.update_controls(V, result.t[i], result.y[:,i]))
        F[i] = forces
        L[i] = levers


    # PLOTTING
    fig1 = plt.figure()
    ax = fig1.add_subplot()
    draw_rocket(ax, V, result.y, spacing=1)
    draw_forces(ax, F, L, result.y, scale=20., colors=['blue', 'orange', 'orange', 'black'],spacing=1)
    draw_traj(ax, result.t, result.y, cmap=plt.cm.Reds)
    ax.axis('equal')
    ax.grid(True)

    draw_states(result.t, result.y)

    plt.show()