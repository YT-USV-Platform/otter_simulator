#!/usr/bin/env python
# -*- coding: utf-8 -*-
from otter import Otter
import numpy as np

if __name__ == "__main__":
    print("Hello World!")

    vehicle = Otter()

    # u = vehicle.stepInput(t=10)

    t = np.linspace(0.0, 10.0, 1000)

    current_eta = np.array([0, 0, 0, 0, 0, 0], float)

    for i in range(len(t)):

        [nu,u_actual] = vehicle.dynamics(
                                eta=current_eta,    # position/attitude, user editable
                                nu=vehicle.nu,
                                u_actual=vehicle.u_actual,
                                u_control=vehicle.stepInput(0.1),
                                sampleTime=t[i])


        print("Nu actual: ", nu)
        print("U actual: ", u_actual)




