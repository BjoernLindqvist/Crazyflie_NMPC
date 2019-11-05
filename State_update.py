def stateupdate(x, u):
    import math
    import numpy as np
    dt = 0.05
    ns = 8
    noise = np.random.normal(0,0.01, ns)
    x[0] = x[0] + dt * x[3] + noise[0]
    x[1] = x[1] + dt * x[4] + noise[1]
    x[2] = x[2] + dt * x[5] + noise[2]
    x[3] = x[3] + dt * (math.sin(x[7]) * math.cos(x[6]) * u[0] - 0.1 * x[3]) + 0.5*noise[3]
    x[4] = x[4] + dt * (-math.sin(x[6]) * u[0] - 0.1*x[4]) + 0.5*noise[4]
    x[5] = x[5] + dt * (math.cos(x[7]) * math.cos(x[6]) * u[0] - 0.2 * x[5] - 9.81) + 0.5*noise[5]
    x[6] = x[6] + dt * ((1 / 0.5) * (u[1] - x[6])) + 0.1*noise[6]
    x[7] = x[7] + dt * ((1 / 0.5) * (u[2] - x[7])) + 0.1*noise[7]
    return(x)
