import opengen as og
import casadi.casadi as cs
import numpy as np
import json
import logging

## Problem size

N = 40
dt = 1.0/20
nMAV = 1  #Number of MAVs to be included in the centralized scheme

## Weight matrices
Qx = (3,3, 60, 5, 5, 3, 8, 8)    
P = 2*Qx; #final state weight
Ru = (5, 10, 10); # input weights
Rd = (10, 25, 25); # input rate weights


## Objective function generation
nu = 3; #Number of control inputs per MAV
ns = 8; #Number of states per MAV
obsparam = 4;
np = nMAV*ns + nMAV*ns + nu + nu*nMAV + obsparam;
#print(np)
u = cs.SX.sym('u', nu*nMAV*N)
z0 = cs.SX.sym('z0', np)
print(z0)
x = z0[0:nMAV*ns]
#print(x)
x_ref = z0[nMAV*ns:nMAV*ns + nMAV*ns]
#print(x_ref)
u_ref = z0[nMAV*ns + nMAV*ns:nMAV*ns + nMAV*ns + nu]
#print(u_ref)
u_old = z0[nMAV*ns + nMAV*ns + nu:nMAV*ns + nMAV*ns + nu + nMAV*nu]
obs_data = z0[nMAV*ns + nMAV*ns + nu + nMAV*nu:nMAV*ns + nMAV*ns + nu + nMAV*nu + obsparam]
#print(obs_data)
cost = 0
c = 0

for i in range(0, N):
###State Cost 
    for j in range(0,nMAV):
        #print(j)
        cost += Qx[0]*(x[ns*j]-x_ref[ns*j])**2 + Qx[1]*(x[ns*j+1]-x_ref[ns*j+1])**2 + Qx[2]*(x[ns*j+2]-x_ref[ns*j+2])**2 + Qx[3]*(x[ns*j+3]-x_ref[ns*j+3])**2 + Qx[4]*(x[ns*j+4]-x_ref[ns*j+4])**2 + Qx[5]*(x[ns*j+5]-x_ref[ns*j+5])**2 + Qx[6]*(x[ns*j+6]-x_ref[ns*j+6])**2 + Qx[7]*(x[ns*j+7]-x_ref[ns*j+7])**2

####Input Cost 
    u_n = u[(i*nMAV*nu):(i*nMAV*nu+nu*nMAV)]
    for j in range(0,nMAV):
        print(j)
        cost += Ru[0]*(u_n[nu*j] - u_ref[0])**2 + Ru[1]*(u_n[nu*j+1] - u_ref[1])**2 + Ru[2]*(u_n[nu*j+2] - u_ref[2])**2 #Input weights
        cost += Rd[0]*(u_n[nu*j] - u_old[nu*j])**2 + Rd[1]*(u_n[nu*j+1] - u_old[nu*j+1])**2 + Rd[2]*(u_n[nu*j+2] - u_old[nu*j+2])**2 #Input rate weights

######Penalty constraint
    #if nMAV > 1:
    #    for j in range(0,nMAV-1):     
    #        c = cs.vertcat(c, cs.fmax(0, 0.4**2 - ((x[ns*j] - x[ns*j+8])**2 + (x[ns*j+1] - x[ns*j+9])**2))) 
 
    #cost += 100*cs.fmax(0, 0.3**2 - ((x[0] - x[8])**2 + (x[1] - x[9])**2))
    #cost += 100*cs.fmax(0, 0.3**2 - ((x[16] - x[24])**2 + (x[17] - x[25])**2))
####Obstacle(hoop)
    for j in range(0,nMAV):     
        c = cs.vertcat(c,obs_data[3] - (x[ns*j]-obs_data[0])**2 - (x[ns*j+1]-obs_data[1])**2)
        #c = cs.vertcat(c,cs.fmax(0, obs_data[3]**2-(x[ns*j]+7)**2 - (x[ns*j+1]-1.5)**2))
        #c = cs.vertcat(c,cs.fmax(0, obs_data[3]**2-(x[ns*j]+7)**2 - (x[ns*j+1]+1.5)**2))
        #c = cs.vertcat(c,cs.fmax(0, obs_data[3]**2-(x[ns*j]+11)**2 - (x[ns*j+1])**2))
        #c = cs.vertcat(c, cs.fmax(0, -0.005 + (u_n[nu*j+1] - u_old[nu*j+1])**2))
        #c = cs.vertcat(c, cs.fmax(0, -0.005 + (u_n[nu*j+2] - u_old[nu*j+2])**2))
        #c = cs.vertcat(c,cs.fmax(0,-(obs_data[3]**2 - ((x[1]-obs_data[1])**2 + (x[2]-obs_data[2])**2))) * cs.fmax(0, (x[0] - obs_data[0]) + 0.4) * cs.fmax(0, -(x[0] - obs_data[0]) + 0.4))
        #c = cs.vertcat(c, -(obs_data[3]**2 - ((x[1]-obs_data[1])**2 + (x[2]-obs_data[2])**2)) * ( (x[0] - obs_data[0]) + 0.4) * (-(x[0] - obs_data[0]) + 0.4))
####State update MAV1
    u_old = u_n
    for j in range(0,nMAV):
        x[ns*j] = x[ns*j] + dt * x[ns*j+3]
        x[ns*j+1] = x[ns*j+1] + dt * x[ns*j+4]
        x[ns*j+2] = x[ns*j+2] + dt * x[ns*j+5]
        x[ns*j+3] = x[ns*j+3] + dt * (cs.sin(x[ns*j+7]) * cs.cos(x[ns*j+6]) * u_n[nu*j] - 0.1 * x[ns*j+3])
        x[ns*j+4] = x[ns*j+4] + dt * (-cs.sin(x[ns*j+6]) * u_n[nu*j] - 0.1*x[ns*j+4])
        x[ns*j+5] = x[ns*j+5] + dt * (cs.cos(x[ns*j+7]) * cs.cos(x[ns*j+6]) * u_n[nu*j] - 0.2 * x[ns*j+5] - 9.81)
        x[ns*j+6] = x[ns*j+6] + dt * ((1 / 0.5) * (u_n[nu*j+1] - x[ns*j+6]))
        x[ns*j+7] = x[ns*j+7] + dt * ((1 / 0.5) * (u_n[nu*j+2] - x[ns*j+7]))
        #print(x[ns*j])
    
smin = [-1000]*N
smax = [0] * N
umin = [5, -0.2, -0.2] * (N*nMAV)
umax = [13.5, 0.2, 0.2] * (N*nMAV)
bounds = og.constraints.Rectangle(umin, umax)
set1 = og.constraints.Rectangle(smin, smax) 
set2 = [-1000,0]*(2*N)
set3 = og.constraints.Ball2(None, 1)


problem = og.builder.Problem(u, z0, cost) \
    .with_aug_lagrangian_constraints(c,set1) \
    .with_constraints(bounds)

build_config = og.config.BuildConfiguration()  \
    .with_build_directory("MAVsim") \
    .with_build_mode("release") 
#.with_build_c_bindings()


meta = og.config.OptimizerMeta()       \
    .with_optimizer_name("multiple_n_alm")
#.with_rebuild(True)


solver_config = og.config.SolverConfiguration()   \
            .with_tolerance(1e-5)                 \
            .with_initial_tolerance(1e-5)         \
            .with_initial_penalty(10)             \
            .with_penalty_weight_update_factor(10) \
            .with_max_outer_iterations(4)



builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
.with_verbosity_level(logging.INFO)
builder.enable_tcp_interface()

builder.build()

# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('MAVsim/multiple_n_alm')
mng.start()
x0 =   [2.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0]*nMAV
#x02=  [1.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0]
xref= [-2.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0]*nMAV
#xref2=[-2.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0]
uold =[9.81,0.0,0.0]*nMAV
uref =[9.81,0.0,0.0]
obsdata = [-1, 0, 0, 0.5]
z0 = x0 + xref + uref + uold + obsdata
print(z0)
print(len(z0)) ##Length is equal to np
#obsdata = (0.0,0.0,1.0,1.0)
mng.ping()
solution = mng.call(z0, initial_guess=[0.0]*(nu*N*nMAV),buffer_len = 4096)
print(solution)
mng.kill()

