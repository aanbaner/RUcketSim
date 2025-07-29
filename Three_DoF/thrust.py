"""
Module managing thrust sources for the rocket simulation.
This module provides functionality to handle thrust sources, including
solid motors, with data taken in from a CSV file.

Author:
    Aarush Banerjee
"""

import pandas as pd
import numpy as np
import config as C
from scipy.integrate import trapezoid
from scipy.interpolate import PPoly, make_interp_spline
import math

class ThrustSource:
    """
    Generic sources of thrust that each thrust source class must extend
    """

    def center_of_thrust(self) -> list[float]:
        """
        Every ThrustSource class should have a center_of_thrust() method that acts as a getter for its center of thrust.
        """
        raise Exception("There is no associated center_of_thrust() method for this ThrustSource object. Rewrite this function in the class that extends ThrustSource")

# Thrust sources subclasses here

class SolidMotor(ThrustSource):
    """
    Class representing a solid motor thrust source.
    """

    def __init__(self, csv_filename: str, 
        CoT: list[float] = C.CoT, 
        qty: int = 1,
        skiprows: int = 3, # for most CSVs downloaded off of thrustcurve.org, skiprows=3 will get rid of the title, link, and author lines, leaving just the numbers
        interp_order: int = C.THRUST_INTERP_ORDER, 
        characteristic_thrust: float = None
    ):
        """
        Creates a solid motor object and adds it to the list of thrust sources of the rocket
        Solid motor objects are created from a CSV file containing thrust data. 
        Thrust curve data can be found from https://www.thrustcurve.org or from the manufacturer.

        Args:
            csv_filename (str): The filepath of the CSV file containing the thrust data.
            qty (int, optional): The number of solidmotors of this type to cluster together. Magnifies thrust.
            skiprows (int, optional): The number of rows to skip at the beginning of the CSV file. Defaults to 3.
            interp_order (int, optional): The order of the spline interpolation used to create the thrust curve. Defaults to const.INTERP_ORDER.
        """
        self.thrust_curve_data: list[list[float]] = [[float(j) for j in i] for i in np.transpose((pd.read_csv(csv_filename)).values.tolist()[skiprows:])]
        self.thrust_curve_data[1] = [qty * thrust for thrust in self.thrust_curve_data[1]]

        # characteristic thrust can be set but defaults to average thrust
        if characteristic_thrust is None:
            self.characteristic_thrust: float = trapezoid(self.thrust_curve_data[1], self.thrust_curve_data[0]) / (self.thrust_curve_data[0][-1] - self.thrust_curve_data[0][0])
        else:
            self.characteristic_thrust: float = characteristic_thrust
        
        self.thrust_curve: PPoly = make_interp_spline(self.thrust_curve_data[0], self.thrust_curve_data[1], k=interp_order)
        self.CoT = CoT
        self.ignition_time: float = math.inf

    def ignite(self, time: float):
        """
        Setter for the ignition_time attribute.

        Args:
            time (float): The time at which the motor is ignited.
        """
        self.ignition_time = time
    
    # all thrust sources should have a thrust_mag method of this signature that returns the thrust magnitude at a given time
    def thrust_mag(self, time: float) -> float:
        """
        This method is used to get the thrust magnitude at a given time.

        Args:
            time (float): The time at which the thrust magnitude is to be calculated.
        """
        return np.vectorize(lambda x: self.thrust_curve(x - self.ignition_time) if (self.ignition_time <= x <= (self.ignition_time + max(self.thrust_curve_data[0]))) else 0.)(time)
    
    def center_of_thrust(self) -> list[float]:
        """
        Getter for the CoT attribute.
        """
        return self.CoT
        
    # GETTERS AND SETTERS
    def get_characteristic_thrust(self) -> float:
        """
        Getter for the characteristic_thrust attribute.
        """
        return self.characteristic_thrust

    def get_thrust_curve_data(self) -> list[list[float]]:
        """
        Getter for the thrust_curve_data attribute calculated from the CSV. Index 0 of the return value is time, index 1 is thrust magnitude.
        """
        return self.thrust_curve_data
    
    def get_thrust_curve(self) -> PPoly:
        """
        Getter for the thrust_curve attribute, a PPoly object constructed from interpolating the thrust data.
        """
        return self.thrust_curve
    
    def get_ignition_time(self) -> float:
        """
        Getter for the ignition_time attribute.
        """
        return self.ignition_time
    
    def get_burntime(self) -> float:
        return self.thrust_curve_data[0][-1]

if __name__ == '__main__':
    
    # Example usage / Testing
    import matplotlib.pyplot as plt

    # These filepaths assume you are running this file from the root directory of the repository
    launch_motor = SolidMotor('ThrustCurves/Estes_E12.csv', qty=3)
    land_motor = SolidMotor('ThrustCurves/AeroTech_H13ST.csv',)

    launch_motor.ignite(5)
    land_motor.ignite(0)

    t_land = np.arange(min(land_motor.get_thrust_curve_data()[0]), 
                       max(land_motor.get_thrust_curve_data()[0]), 
                       0.001)
    t_launch = np.arange(min(launch_motor.get_thrust_curve_data()[0]), 
                         max(launch_motor.get_thrust_curve_data()[0]), 
                         0.001)
    
    plt.rcParams['text.usetex'] = True
    
    plt.subplot(131)
    plt.plot(t_land, launch_motor.thrust_mag(t_land))
    plt.plot(t_land, [launch_motor.get_characteristic_thrust() for i in t_land], color = 'gray', linestyle = '--')
    plt.plot([launch_motor.get_burntime() + launch_motor.get_ignition_time() for i in range(10)], np.linspace(-0, 110, 10), color = 'red', linestyle = '--')
    plt.scatter(np.array(launch_motor.get_thrust_curve_data()[0]) + 5, launch_motor.get_thrust_curve_data()[1], color='orange', marker="X")
    plt.ylabel(r'Thrust (N)')
    plt.xlabel(r'Time (s)')
    plt.title(r'Thrust Curve for Estes E12 (Launch Motor)')
    plt.xlim(min(land_motor.get_thrust_curve_data()[0]), max(land_motor.get_thrust_curve_data()[0]))
    plt.grid()

    plt.subplot(132)
    plt.plot(t_land, land_motor.thrust_mag(t_land))
    plt.plot(t_land, [land_motor.get_characteristic_thrust() for i in t_land], color = 'gray', linestyle = '--')
    plt.plot([land_motor.get_burntime() + land_motor.get_ignition_time() for i in range(10)], np.linspace(-0, 50, 10), color = 'red', linestyle = '--')
    plt.scatter(land_motor.get_thrust_curve_data()[0], land_motor.get_thrust_curve_data()[1], color='orange', marker="X")
    plt.ylabel(r'Thrust (N)')
    plt.xlabel(r'Time (s)')
    plt.title(r'Thrust Curve for AeroTech H13ST (Landing Motor)')
    plt.xlim(min(land_motor.get_thrust_curve_data()[0]), max(land_motor.get_thrust_curve_data()[0]))
    plt.grid()

    plt.subplot(133)
    plt.plot(t_land, launch_motor.thrust_mag(t_land) + land_motor.thrust_mag(t_land))
    plt.plot(t_land, np.array([land_motor.characteristic_thrust for i in t_land]) + np.array([launch_motor.characteristic_thrust for i in t_land]), color = 'gray', linestyle = '--')
    plt.plot([land_motor.get_burntime() + land_motor.get_ignition_time() for i in range(10)], np.linspace(-0, 50, 10), color = 'red', linestyle = '--')
    plt.plot([launch_motor.get_burntime() + launch_motor.get_ignition_time() for i in range(10)], np.linspace(-0, 110, 10), color = 'red', linestyle = '--')
    plt.xlabel(r'Time (s)')
    plt.ylabel(r'Thrust (N)')
    plt.title(r'Overlapping thrust curves')
    plt.xlim(min(land_motor.get_thrust_curve_data()[0]), max(land_motor.get_thrust_curve_data()[0]))
    plt.grid()

    fig, axs = plt.subplots(1,3)
    for i in range(3):
        motor = SolidMotor('ThrustCurves/Estes_E12.csv', interp_order=i+1)
        motor.ignite(0)
        axs[i].plot(t_launch, motor.thrust_mag(t_launch), linewidth=0.8)
        axs[i].scatter(np.array(motor.get_thrust_curve_data()[0]), motor.get_thrust_curve_data()[1], color='orange', marker="X", linewidth=0.5)
        axs[i].set_xlabel(r'Time (t)')
        axs[i].set_ylabel(r'Thrust (N)')
        axs[i].set_title(str(i+1) + " polynomial order spline")

    plt.show()