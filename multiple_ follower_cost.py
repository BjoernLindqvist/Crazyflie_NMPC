import opengen as og
import casadi.casadi as cs
import numpy as np

## Problem size

N = 40
dt = 1.0/20

## Weight matrices
Qx = (10,10, 30, 3, 3, 3, 8, 8)    
P = 2*Qx; #final state weight
Ru = (5, 15, 15); # input weights
Rd = (10,20,20); # input rate weights


## Objective function generation
nu = 6;
np = 41;
print(np)
u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', np)
print(z0)
x = z0[0:8]
x2 = z0[8:16]
x_ref = z0[16:24]
x_ref2 = z0[24:32]
u_ref = z0[32:35]

u_old = z0[35:38]
u_old2 = z0[38:41]

#obs_data = z0[41:]
#print(obs_data)
cost = 0
c = 0

for i in range(0, N):
###State Cost MAV1
    cost += Qx[0]*(x[0]-x_ref[0])**2 + Qx[1]*(x[1]-x_ref[1])**2 + Qx[2]*(x[2]-x_ref[2])**2 + Qx[3]*(x[3]-x_ref[3])**2 + Qx[4]*(x[4]-x_ref[4])**2 + Qx[5]*(x[5]-x_ref[5])**2 + Qx[6]*(x[6]-x_ref[6])**2 + Qx[7]*(x[7]-x_ref[7])**2
###State Cost MAV2
    cost += Qx[0]*(x2[0]-x_ref2[0])**2 + Qx[1]*(x2[1]-x_ref2[1])**2 + Qx[2]*(x2[2]-x_ref2[2])**2 + Qx[3]*(x2[3]-x_ref2[3])**2 + Qx[4]*(x2[4]-x_ref2[4])**2 + Qx[5]*(x2[5]-x_ref2[5])**2 + Qx[6]*(x2[6]-x_ref2[6])**2 + Qx[7]*(x2[7]-x_ref2[7])**2
####Input Cost MAV1
    u_n = u[(6*i):(6*i+3)]
    cost += Ru[0]*(u_n[0] - u_ref[0])**2 + Ru[1]*(u_n[1] - u_ref[1])**2 + Ru[2]*(u_n[2] - u_ref[2])**2 #Input weights
    cost += Rd[0]*(u_n[0] - u_old[0])**2 + Rd[1]*(u_n[1] - u_old[1])**2 + Rd[2]*(u_n[2] - u_old[2])**2 #Input rate weights
####Input Cost MAV2
    u_n2 = u[(6*i+3):(6*i+6)]
    cost += Ru[0]*(u_n2[0] - u_ref[0])**2 + Ru[1]*(u_n2[1] - u_ref[1])**2 + Ru[2]*(u_n2[2] - u_ref[2])**2 #Input weights
    cost += Rd[0]*(u_n2[0] - u_old2[0])**2 + Rd[1]*(u_n2[1] - u_old2[1])**2 + Rd[2]*(u_n2[2] - u_old2[2])**2 #Input rate weights
######Penalty constraint
    #c = cs.vertcat(c, cs.fmax(0, 0.49 - ((x[0] - x2[0])**2 + (x[1] - x2[1])**2))) #(x[2] - x2[2])**2))
    c = cs.vertcat(c, cs.fmax(0, -0.003 + (u_n[1] - u_old[1])**2))
    c = cs.vertcat(c, cs.fmax(0, -0.003 + (u_n[2] - u_old[2])**2))
    c = cs.vertcat(c, cs.fmax(0, -0.003 + (u_n2[1] - u_old2[1])**2))
    c = cs.vertcat(c, cs.fmax(0, -0.003 + (u_n2[2] - u_old2[2])**2))
    u_old = u_n
    u_old2 = u_n2
####Obstacle(hoop)
    #c += cs.fmax(0, 0.49-(x[0]+0.5)**2 - (x[1]-4)**2)
    #c += 0*cs.fmax(0,-(obs_data[3]**2 - ((x[1]-obs_data[1])**2 + (x[2]-obs_data[2])**2))) * cs.fmax(0, (x[0] - obs_data[0]) + 0.4) * cs.fmax(0, -(x[0] - obs_data[0]) + 0.4)
####State update MAV1
    x[0] = x[0] + dt * x[3]
    x[1] = x[1] + dt * x[4]
    x[2] = x[2] + dt * x[5]
    x[3] = x[3] + dt * (cs.sin(x[7]) * cs.cos(x[6]) * u_n[0] - 0.1 * x[3])
    x[4] = x[4] + dt * (-cs.sin(x[6]) * u_n[0] - 0.1*x[4])
    x[5] = x[5] + dt * (cs.cos(x[7]) * cs.cos(x[6]) * u_n[0] - 0.2 * x[5] - 9.81)
    x[6] = x[6] + dt * ((1 / 0.5) * (u_n[1] - x[6]))
    x[7] = x[7] + dt * ((1 / 0.5) * (u_n[2] - x[7]))
#####State update MAV2
    x2[0] = x2[0] + dt * x2[3]
    x2[1] = x2[1] + dt * x2[4]
    x2[2] = x2[2] + dt * x2[5]
    x2[3] = x2[3] + dt * (cs.sin(x2[7]) * cs.cos(x2[6]) * u_n2[0] - 0.1 * x2[3])
    x2[4] = x2[4] + dt * (-cs.sin(x2[6]) * u_n2[0] - 0.1*x2[4])
    x2[5] = x2[5] + dt * (cs.cos(x2[7]) * cs.cos(x2[6]) * u_n2[0] - 0.2 * x2[5] - 9.81)
    x2[6] = x2[6] + dt * ((1 / 0.5) * (u_n2[1] - x2[6]))
    x2[7] = x2[7] + dt * ((1 / 0.5) * (u_n2[2] - x2[7]))


umin = [8, -0.3, -0.3, 8, -0.3, -0.3] * (nu*N)
umax = [13.5, 0.3, 0.3, 13.5, 0.3, 0.3] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)
problem = og.builder.Problem(u, z0, cost).with_penalty_constraints(c) \
.with_constraints(bounds)
build_config = og.config.BuildConfiguration()  \
.with_build_directory("MAV") \
.with_build_mode("release") \
.with_build_c_bindings()
meta = og.config.OptimizerMeta()       \
.with_optimizer_name("multiplefollower2")
#.with_rebuild(True) 
solver_config = og.config.SolverConfiguration() \
.with_tolerance(1e-3) \
.with_max_outer_iterations(5) \
.with_penalty_weight_update_factor(10) \
.with_initial_penalty_weights(10.0) 
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
.with_verbosity_level(1)
builder.enable_tcp_interface()

builder.build()

# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('MAV/multiplefollower2')
mng.start()
x0=   [2.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0]
x02=  [1.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0]
xref= [-2.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0]
xref2=[-2.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0]
uold =[9.81,0.0,0.0]
uold2 =[9.81,0.0,0.0]
uref =[9.81,0.0,0.0]
z0 = x0 + x02 + xref + xref2 + uref + uold + uold2 
print(z0)
print(len(z0)) ##Length is equal to np
#obsdata = (0.0,0.0,1.0,1.0)
mng.ping()
solution = mng.call(z0, initial_guess=[0.0]*(nu*N),buffer_len = 4*4096)
print(solution)
mng.kill()

