from math import pi

# configuration constants
THRUST_INTERP_ORDER = 1 # order of the spline interpolation for the thrust curve

# vehicle-specific constants
    # inertia
m: float = 0.9 # mass of pure vehicle (no prop) : kg
J: float = 0.1 # moment of inertia of pure vehicle (no prop) : kg*m^2
    # 2D positions of the centers of the vehicle
CoG: list[float] = [0, 0] # center of gravity
CoP: list[float] = [0.1, 0.01] # center of pressure
CoT: list[float] = [-0.2, 0] # center of thrust
base: list[float] = [-0.2, 0] # base of the rocket (only used for graphics)
    # geometry
length: float = 0.6096 # 2ft (m)
dia: float = 0.096 # 96mm (m)
    # actuator limits
        # gimbal
max_gimbal_angle: float = pi/9 # 20 degrees
max_gimbal_angle_rate: float = 0.5 # rad/s
        # throttle
max_thrust: float = 0.5 # N
min_thrust: float = 0.1 # N
max_thrust_rate: float = 1 # N/s
    # aerodynamic constants
CD_0: float = 0.75 # straight flights
CD_flat: float = 1.28 # flat plate estimation
CD_thetadot: float = 0.63 # coefficient of moment of drag due to angular speed
CL_alpha: float = 0.06 # lift coefficient

# Environmental Constants
    # mini gravity
g: float = 9.81 # m/s^2
    # Newtonian gravity
G: float = 6.6743 * (10**-11)
M: float = 5.972 * (10**24)
R: float = 6378 * (10**3)
    # aerial constants
rho: float = 1.293 # kg/m^3
temp: float = 273.15 # Kelvin
P: float = 101325 # pascals
# Enabling/Disabling
ENABLE_GRAVITY = True
ENABLE_DRAG = True
ENABLE_CONTROL = True