"""
top-level example

Author:
    Aarush Banerjee
"""

import numpy as np
import matplotlib.pyplot as plt
import config as C
import vehiclemodel as vm
import index
from scipy.integrate import solve_ivp
from draw import draw_rocket, draw_traj, draw_forces, draw_states
from math import pi
from dynamics import calc_forces_levers_moments
from controller import update_controls

# time array
dT = 0.01
simtime = 100
T = np.linspace(0, simtime, int(simtime / dT) + 1)

# data construction
V = vm.Vehicle(rho=.1)
launch = V.SolidMotor('ThrustCurves/Estes_E12.csv', qty=4) # index 0
land = V.SolidMotor('ThrustCurves/AeroTech_F13-RCT.csv') # index 1

# initial conditions
X0: list[float] = [0, 0, pi/2, 0, 0, 0, 0]

# inground event to terminate solver
def inGroundEvent(t: float, y: list[float]) -> bool:
    return not (y[index.Y] <= 0 and y[index.Y_DOT] < 0  and t > 3)
inGroundEvent.terminal = True

# solve
result = solve_ivp(V.update, [T[0], T[-1]], X0, method='DOP853', t_eval=T, events=inGroundEvent)

# data post processing
    # get forces and levers for drawing
F = np.zeros((len(result.t), 2 + len(V.thrust_sources), 2)) 
L = np.zeros((len(result.t), 2 + len(V.thrust_sources), 2))
for i in (range(len(result.t))):
    forces, levers, moments = calc_forces_levers_moments(V, result.t[i], result.y[:,i], update_controls(V, result.t[i], result.y[:,i]))
    F[i] = forces
    L[i] = levers

thr = launch.thrust_mag(result.t) + land.thrust_mag(result.t)

# Terminal prints
if (result.y[index.Y, -1] <= 0):
    i = np.argmin(np.abs(result.y[index.Y, 10:])) # find where the trajectory hits y=0
    print("Vehicle impacted ground at time t = " + str(result.t[i]) + ", where state = " + str(result.y[:,i]) + " and energy = " + str((0.5 * V.m) * (result.y[index.X_DOT,i]**2 + result.y[index.Y_DOT,i]**2)) + " Joules")
else:
    print("Simulation timed out at t = " + str(result.t[-1]) + ", where state = " + str(result.y[:,-1]))
print("Shape of the resulting state array: " + str(np.shape(result.y)))
print("Launch motor ignited at time = " + str(launch.ignition_time))
print("Landing motor ignited at time = " + str(land.ignition_time))
print("Cost function evaluated over launch time = " + str())

# PLOTTING
plt.rcParams['text.usetex'] = True
fig1 = plt.figure()
ax = fig1.add_subplot()
draw_rocket(ax, V, result.y, spacing=5)
draw_forces(ax, F, L, result.y, scale=20., colors=['blue', 'orange', 'orange', 'black'],spacing=5)
draw_traj(ax, result.t, result.y, cmap=plt.cm.Reds)
ax.set_xlabel(r'$x$ (m)')
ax.set_ylabel(r'$y$ (m)')
ax.axis('equal')
ax.grid(True)
ax.set_title(r'Trajectory')

# controls responses plot
fig2 = plt.figure()
    # data processing
ax2 = fig2.add_subplot()
theta_comm = pi/2
theta = [(theta % (2*pi)) - (2*pi) if (theta % (2*pi)) > (theta_comm + pi) else (theta % (2*pi)) + (2*pi) if (theta % (2*pi)) < (theta_comm - pi) else (theta % (2*pi)) for theta in result.y[index.THETA]]
ctrl_thru = [max(-1 * thr[i], min(1 * thr[i], 1 * update_controls(V, result.t[i], result.y[:,i])[0])) 
                    for i in range(len(result.t))]
upper = np.max(np.array([result.y[index.THETA_DOT], theta, ctrl_thru]))
lower = np.min(np.array([result.y[index.THETA_DOT], theta, ctrl_thru]))
    # plotting
ax2.plot(result.t, theta, label = r'$\theta$ ($rad$)')
ax2.plot(result.t, result.y[index.THETA_DOT], color = 'tab:blue', alpha = 0.4, label = r'$\dot{\theta}$ ($\frac{{rad}}{{s}}$)')
ax2.plot(result.t, ctrl_thru, label = r'$u$ ($N$)')
        # commanded theta
ax2.plot(result.t, [theta_comm for i in result.t], color = 'black', linestyle = ':', label = r'$\theta$ commanded')
        # ignition/burnout times
ax2.plot([launch.ignition_time]*2, [lower, upper], color="black", linestyle='--', label = 'launch ignition and burnout times', linewidth = 0.9)
ax2.plot([launch.ignition_time + launch.get_burntime()]*2, [lower, upper], color='black', linestyle='--', linewidth = 0.9)
ax2.plot([land.ignition_time]*2, [lower, upper], color="#696969", linestyle='--', label = 'land ignition and burnout times', linewidth = 0.9)
ax2.plot([land.ignition_time + land.get_burntime()]*2, [lower, upper], color='#696969', linestyle='--', linewidth = 0.9)

ax2.set_yticks(np.arange(lower - (lower % (pi/2)), upper - (upper % (pi/2)) + (pi/2), pi/2))
ax2.legend(loc='upper center')
ax2.grid(True)
ax2.set_title(r'Inner Loop Control Response')
ax2.set_xlabel(r'Time ($s$)')
ax2.set_xticks(np.arange(0,land.get_ignition_time() + land.get_burntime(), 5))

draw_states(result.t, result.y, theta_comm)

fig3 = plt.figure()
ax3 = fig3.add_subplot()
ax3.plot(result.t, thr, label = 'thrust vector magnitude')
ax3.plot([result.t[0], result.t[-1]], [V.m * C.g, V.m * C.g], linestyle = '--', label = 'magnitude of gravity')
ax3.set_title(r'Magnitude of Thrusters Over Time')
ax3.set_xlabel(r'Time (s)')
ax3.set_ylabel(r'Force Magnitude (N)')
ax3.plot([launch.ignition_time]*2, [0, 100], color='green', linestyle='--', label = 'launch ignition and burnout times', linewidth = 0.9)
ax3.plot([launch.ignition_time + launch.get_burntime()]*2, [0, 100], color='green', linestyle='--', linewidth = 0.9)
ax3.plot([land.ignition_time]*2, [0, 100], color='red', linestyle='--', label = 'land ignition and burnout times', linewidth = 0.9)
ax3.plot([land.ignition_time + land.get_burntime()]*2, [0, 100], color='red', linestyle='--', linewidth = 0.9)
ax3.grid(True)
ax3.legend(loc='upper right')

plt.show()