import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

## Problem size

N = 40
dt = 1.0/20

## Weight matrices
Qx = (4,4, 30, 3, 3, 3, 8, 8)    
P = 2*Qx; #final state weight
Ru = (5, 10, 10); # input weights
Rd = (5,12,12); # input rate weights

## Objective function generation
nu = 3;
np = 26;
u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', np)
x = z0[0:8]
print(x)
x_ref = z0[8:16]
print(x_ref)
u_ref = z0[16:19]
print(u_ref)
u_old = z0[19:22]
print(u_old)
obs_data = z0[22:]
print(obs_data)
cost = 0
c = 0

for i in range(0, 40):
    #for t in range(0 , 8):
    #    cost +=Qx[t]*(x[t]-x_ref[t])**2
    cost += Qx[0]*(x[0]-x_ref[0])**2 + Qx[1]*(x[1]-x_ref[1])**2 + Qx[2]*(x[2]-x_ref[2])**2 + Qx[3]*(x[3]-x_ref[3])**2 + Qx[4]*(x[4]-x_ref[4])**2 + Qx[5]*(x[5]-x_ref[5])**2 + Qx[6]*(x[6]-x_ref[6])**2 + Qx[7]*(x[7]-x_ref[7])**2  #State weights

    u_n = u[(3*i):3*i+3]
    cost += Ru[0]*(u_n[0] - u_ref[0])**2 + Ru[1]*(u_n[1] - u_ref[1])**2 + Ru[2]*(u_n[2] - u_ref[2])**2 #Input weights
    cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2 #Input rate weights
    u_old = u_n

#####PENALTY CONSTRAINTS
    #Obstacle(hoop)
    #c += cs.fmax(0, 0.49-(x[0]+0.5)**2 - (x[1]-4)**2)
    #c += cs.fmax(0, -(obs_data[3]**2 - ((x[1]-obs_data[1])**2 + (x[2] - obs_data[2])**2))) * cs.fmax(0, (x[0] - obs_data[0]) + 0.4) * cs.fmax(0, -(x[0] - obs_data[0]) + 0.4)
    c = cs.vertcat(c, 10*cs.fmax(0,-(obs_data[3]**2 - ((x[1]-obs_data[1])**2 + (x[2]-obs_data[2])**2))) * cs.fmax(0, (x[0] - obs_data[0]) + 0.4) * cs.fmax(0, -(x[0] - obs_data[0]) + 0.4))
    c = cs.vertcat(c, cs.fmax(0, -0.003 + (u_n[1] - u_old[1])**2))
    c = cs.vertcat(c, cs.fmax(0, -0.003 + (u_n[2] - u_old[2])**2))
    #######State update
    x[0] = x[0] + dt * x[3]
    x[1] = x[1] + dt * x[4]
    x[2] = x[2] + dt * x[5]
    x[3] = x[3] + dt * (cs.sin(x[7]) * cs.cos(x[6]) * u_n[0] - 0.1 * x[3])
    x[4] = x[4] + dt * (-cs.sin(x[6]) * u_n[0] - 0.1*x[4])
    x[5] = x[5] + dt * (cs.cos(x[7]) * cs.cos(x[6]) * u_n[0] - 0.2 * x[5] - 9.81)
    x[6] = x[6] + dt * ((1 / 0.5) * (u_n[1] - x[6]))
    x[7] = x[7] + dt * ((1 / 0.5) * (u_n[2] - x[7]))

cost += 3*Qx[0]*(x[0]-x_ref[0])**2 + Qx[1]*(x[1]-x_ref[1])**2 + Qx[2]*(x[2]-x_ref[2])**2 + Qx[3]*(x[3]-x_ref[3])**2 + Qx[4]*(x[4]-x_ref[4])**2 + Qx[5]*(x[5]-x_ref[5])**2 + Qx[6]*(x[6]-x_ref[6])**2 + Qx[7]*(x[7]-x_ref[7])**2
umin = [5, -0.3, -0.3] * (nu*N)
#print(umin)
umax = [13.5, 0.3, 0.3] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)
problem = og.builder.Problem(u, z0, cost).with_penalty_constraints(c).with_constraints(bounds)
build_config = og.config.BuildConfiguration()  \
.with_build_directory("MAV") \
.with_build_mode("release") \
.with_build_c_bindings()
meta = og.config.OptimizerMeta()       \
.with_optimizer_name("hoop") 
#.with_rebuild(True) 
solver_config = og.config.SolverConfiguration() \
.with_tolerance(1e-3) \
.with_max_outer_iterations(4) \
.with_penalty_weight_update_factor(10) \
.with_initial_penalty_weights(10.0) 
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
.with_verbosity_level(1)
builder.enable_tcp_interface()

builder.build()
# Use TCP server
# ------------------------------------
#mng = og.tcp.OptimizerTcpManager('MAV/hoop')
#mng.start()
#x0 = (1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0)
#xref = (1.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0)
#uold = (9.81,0.0,0.0)
#uref = (9.81,0.0,0.0)
#obsdata = (0.0,0.0,1.0,1.0)
#mng.ping()
#solution = mng.call([x0, xref, uref, uold, obsdata], initial_guess=[1]*(120))
#print(solution)
#mng.kill()


