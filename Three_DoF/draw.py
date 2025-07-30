"""
Module containing functions for plots of trajectory, state variables, etc.

Author:
    Aarush Banerjee
"""

import config as C
import vehiclemodel as vm
import matplotlib.pyplot as plt
import numpy as np
import index
import rotations as rot
from math import pi

def draw_rocket(ax: plt.Axes, V: vm.Vehicle, y: list[list[float]], color = 'gray', spacing: int = 1):

    X = np.array(y)[:,0:-1:spacing]

    bases = np.transpose(rot.rot_bi(X[index.THETA], [V.base for i in X[0]])) + X[index.X: index.Y + 1, :]
    vectors = np.transpose(rot.rot_bi(X[index.THETA], [[V.length, 0] for i in X[0]]))

    ax.quiver(bases[0], bases[1], vectors[0], vectors[1], scale=1., scale_units = 'x', color=color)

def draw_forces(ax: plt.Axes, forces: list[list[list[float]]], centers: list[list[list[float]]], y: list[list[float]], scale: float = 1., colors: list = ['blue', 'orange', 'orange', 'black'], spacing: int = 1):
    F = np.array(forces)[0:-1:spacing]
    C = np.array(centers)[0:-1:spacing]

    X = np.array(y)[:,0:-1:spacing]
    
    for i in range(np.shape(C)[1]):
        thrust_centers = np.transpose(rot.rot_bi(X[index.THETA,:], C[:,i])) + X[index.X:index.Y + 1, :]
        force_vectors = np.transpose(rot.rot_bi(X[index.THETA,:], F[:,i]))
        ax.quiver(thrust_centers[0,:], thrust_centers[1,:], force_vectors[0,:], force_vectors[1,:], scale=scale, scale_units='x', color = 'black' if i >= len(colors) else colors[i])

def draw_traj(ax: plt.Axes, t: list[float], y: list[list[float]], cmap: plt.Colormap = plt.cm.jet):
    # adapted from @askewchan's answer at https://stackoverflow.com/questions/15617207/line-colour-of-3d-parametric-curve-in-pythons-matplotlib-pyplot
    for i in range(len(y[index.X])-1):
        ax.plot(y[index.X][i:i+2], y[index.Y][i:i+2], color=cmap(round(255 * t[i] / max(t))))

def draw_states(t, y, theta_comm):
    fig, axs = plt.subplots(2,3)
    [[axes.grid(True) for axes in i] for i in axs]
    [[axes.set_xlabel(r"$t$") for axes in i] for i in axs]
    axs[0,0].plot(t, y[index.X])
    axs[0,0].set_ylabel(r"$x$")
    axs[0,1].plot(t, y[index.Y])
    axs[0,1].set_ylabel(r"$y$")
    axs[0,2].plot(t, [(theta % (2*pi)) - (2*pi) if (theta % (2*pi)) > (theta_comm + pi) else (theta % (2*pi)) + (2*pi) if (theta % (2*pi)) < (theta_comm - pi) else (theta % (2*pi)) for theta in y[index.THETA]])
    axs[0,2].set_ylabel(r"$\theta$")
    axs[1,0].plot(t, y[index.X_DOT], 'tab:red')
    axs[1,0].set_ylabel(r"$\dot x$")
    axs[1,1].plot(t, y[index.Y_DOT], 'tab:red')
    axs[1,1].set_ylabel(r"$\dot y$")
    axs[1,2].plot(t, y[index.THETA_DOT], 'tab:red')
    axs[1,2].set_ylabel(r"$\dot\theta$")

    upper = theta_comm + pi
    lower = theta_comm - pi
    axs[0,2].set_yticks(np.arange(lower - (lower % (pi/2)), upper - (upper % (pi/2)) + (pi/2), pi/2))

if __name__ == '__main__':

    V = vm.Vehicle()

    fig = plt.figure()
    ax = fig.add_subplot()
    ax.axis('equal')

    t = np.linspace(0, 10, 101)


    x = t
    y =  np.sin(t)
    theta = pi/2 * t
    xdot = np.ones_like(t)
    ydot = np.ones_like(t)
    thetadot = 0.1 * pi/2 * np.ones_like(t)

    y = np.array([x,y,theta,xdot,ydot,thetadot])

    centers: list[list[list[float]]] = []
    forces: list[list[list[float]]] = []
    for i in range(len(t)):
        fg = rot.R_ib(theta[i]) @ np.array([0, -0.5])
        centers.append([[-0.2, 0.], [0.4, 0.], [0., 0.]])
        forces.append([[0.1, 0.], [0., 0.2], fg.tolist()])

    draw_rocket(ax, V, y, color='gray',  spacing=1)
    draw_rocket(ax, V, y, color='red',   spacing=3)
    draw_rocket(ax, V, y, color='green', spacing=5)
    
    draw_forces(ax, forces, centers, y, spacing=3)

    draw_traj(ax, t, y, cmap=plt.cm.plasma)

    plt.show()