#!/usr/bin/env python
# -*- coding: utf-8 -*-
from otter import Otter
import numpy as np
import matplotlib.pyplot as plt


legendSize = 10  # legend size
figSize1 = [25, 13]  # figure1 size in cm
figSize2 = [25, 13]  # figure2 size in cm
dpiValue = 150  # figure dpi value

sampleTime = 0.01  # sample time
stopTime = 10  # stop time


if __name__ == "__main__":
    print("Hello World!")

    vehicle = Otter()

    # u = vehicle.stepInput(t=10)

    t = np.linspace(0.0, stopTime, stopTime*1/sampleTime)

    vehicle.nu = np.array([0, 0, 0, 0, 0, 0], float)
    vehicle.u_actual = np.array([0, 0], float)
    vehicle.u_control = np.array([0, 0], float)
    
    current_eta = np.array([0, 0, 0, 0, 0, 0], float)

    eta_list = np.zeros((6, len(t)-1), float)
    u_control_list = []
    u_actual_list = []
    for i in range(len(t)-1):

        vehicle.u_control = vehicle.stepInput(i, len(t)-1)

        [nu,u_actual] = vehicle.dynamics(
                                eta=current_eta,    # position/attitude, user editable
                                nu=vehicle.nu,
                                u_actual=vehicle.u_actual,
                                u_control=vehicle.u_control,
                                sampleTime=sampleTime)


        print("-----------------------")
        print("Iteration: ", i)
        print("Nu actual: ", nu)
        print("U_control: ", vehicle.u_control)
        print("U actual: ", u_actual)
        vehicle.nu = nu
        vehicle.u_actual = u_actual

        current_eta = current_eta + nu*(t[i+1]-t[i])

        eta_list[:,i] = np.transpose(current_eta)
        print("Current eta: ", current_eta)

        u_control_list.append(vehicle.u_control)
        u_actual_list.append(vehicle.u_actual)



    # create a figure
    plt.figure(1)
    plt.plot(t[:-1], u_control_list)
    plt.plot(t[:-1], u_actual_list)
    plt.legend(["Control Signal 1", "Control Signal 2", 
                "Motor 1 Rad/s", "Motor 2 Rad/s"], fontsize=legendSize)
    plt.xlabel("Time (s)")
    plt.ylabel("Control Signal (rad/s)")
    plt.grid()


    plt.figure(2)
    plt.plot(t[:-1], eta_list[:][0])   # X position
    plt.plot(t[:-1], eta_list[:][1])   # Y position
    plt.legend(["X Position of Vehicle", "Y Position of Vehicle"], fontsize=legendSize)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.grid()

    plt.figure(3)
    plt.plot(t[:-1], eta_list[:][5])   # Yaw angle
    plt.legend(["Yaw Angle of Vehicle"], fontsize=legendSize)
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw Angle (rad)")
    plt.grid()

    plt.show()



