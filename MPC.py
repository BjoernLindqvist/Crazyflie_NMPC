import casadi.casadi as cs
import opengen as og
import numpy as np
import math

N=40
dt=1.0/10.0
nu=3 #number of inputs
nx=8 #number of states
nf=3  #number of external forces (decision variables)
no=4 #number of obstacles
n_dsafety=1;
nu_hover=3;
nQ_state=8;
nQ_hovering=3;
nQ_u=3;
nQ_Obstacle = 4
nCylinders = 4*3
nObslist=4;


cost=0;
u = cs.SX.sym("u", N*nu) # decision variable (nu = 3 T theta_x theta_y)
p = cs.SX.sym("p", nx+nf+nx+no+n_dsafety+ nu_hover+nQ_state+ nQ_hovering+ nQ_u+nQ_Obstacle + nCylinders + nObslist) # states + state_ref (np = 8 p p_dot theta_x theta_y) + obstacles
X= p[0:nx]
f_e=p[nx:nx+nf]
x_ref = p[nx+nf:nx+nf+nx]
x_obst= p[nx+nf+nx:nx+nf+nx+no] # 0: +x 1:-x 2:+y 3: -y 
d_safe= p[nx+nf+nx+no:nx+nf+nx+no+n_dsafety] # [m]

## weights and hovering reference
u_hover= p[nx+nf+nx+no+n_dsafety:nx+nf+nx+no+n_dsafety+nu_hover]#[9.81,0,0]
Q_state= p[nx+nf+nx+no+n_dsafety+nu_hover:nx+nf+nx+no+n_dsafety+nu_hover+nQ_state]#[10.0,10.0,10.0,5.0,5.0,5.0,1.0,1.0];
Q_hovering=p[nx+nf+nx+no+n_dsafety+nu_hover+nQ_state:nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering]#[10.0,10.0,10.0];
Q_u=p[nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering:nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering+nQ_u]#[20,20,20];
Q_obstacle=p[nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering+nQ_u:nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering+nQ_u+nQ_Obstacle]#[1.0,1.0,1.0,1.0];  # 0: +x 1:-x 2:+y 3: -y
Q_Cylinders = p[nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering+nQ_u + nQ_Obstacle:nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering+nQ_u+nQ_Obstacle + nCylinders]
Q_Obslist = p[nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering+nQ_u + nQ_Obstacle + nCylinders:nx+nf+nx+no+n_dsafety+nu_hover+nQ_state+nQ_hovering+nQ_u+nQ_Obstacle + nCylinders + nObslist]
print('X',X)
print('x_ref',x_ref)
print('x_obst',x_obst)
print('d_safe',d_safe)
print('u_hover',u_hover)
print('Q_state',Q_state)
print('Q_hovering',Q_hovering)
print('Q_u',Q_u)
print('Q_Obstacle',Q_obstacle)
print(Q_Cylinders)
print(Q_Obslist)


## Constants
g=9.81 # %gravity
mass = 1.1# % of platform, in kilograms
max_thrust = 30# % of platform, in Newtons

a_x=0.1# % Mass normalized drag coefficients
a_y=0.1#
a_z=0.2#

tao_roll=0.5 #; % Time constants in seconds
tao_pitch=0.5 #;

k_roll=1 ##; % Angle gains
k_pitch=1 #;
x0=X

# bounds on input
u_min = [0,-math.pi/4,-math.pi/4]
u_max = [30,math.pi/4,math.pi/4]
umin=np.kron(np.ones((1,N)),u_min)
umin=umin.tolist()
umin=umin[0]
umax=np.kron(np.ones((1,N)),u_max)
umax=umax.tolist()
umax=umax[0]
c=[]

for i in range(N) :
    k = (i ) * nu
    u0=u[k:k+nu]
    p_0=x0[0:3]#; % position vector
    v=x0[3:6]#; % velocity vector
    roll=x0[6]#; % roll angle
    pitch=x0[7]#; % pitch angle
    T = u0[0]#; % mass normalized thrust
    roll_g = u0[1]#; %desired angles
    pitch_g = u0[2]#;

    #R_x = np.matrix([[1, 0, 0],[ 0, np.cos(roll), -np.sin(roll)],[0, np.sin(roll), np.cos(roll)]])#; %Rotation around x (roll)
    #R_y = np.matrix([[np.cos(pitch), 0, np.sin(pitch)],[ 0, 1, 0],[-np.sin(pitch), 0, np.cos(pitch)]])#; %Rotation around y (pitch)
    #R = R_y*R_x#; % Rotation matrix without rotation around z since coordinates are fixed to the frame
    #x_0 = p_0[0]
    #y_0 = p_0[1]
    #z_0 = p_0[2]
    #v_dot = R*np.matrix([[0],[ 0],[T]]) + np.matrix([[0],[ 0],[-g]]) - np.matrix([[a_x, 0, 0],[ 0,a_y, 0],[ 0, 0, a_z]])*v
    ddx= (cs.sin(pitch)*cs.cos(roll))*T-a_x*x0[3]+f_e[0]
    ddy= (-cs.sin(roll))*T-a_y*x0[4]+f_e[1]
    ddz=-g+(cs.cos(pitch)*cs.cos(roll))*T-a_z*x0[5]+f_e[2]

    roll_dot = (1/tao_roll)*(k_roll*roll_g - roll)

    pitch_dot = (1/tao_pitch)*(k_pitch*pitch_g - pitch)

    x0[0]+=x0[3]*dt
    x0[1]+=x0[4]*dt
    x0[2]+=x0[5]*dt
    x0[3]+=ddx*dt
    x0[4]+=ddy*dt
    x0[5]+=ddz*dt
    x0[6]+=roll_dot*dt
    x0[7]+=pitch_dot*dt
    # Obstacle dynamics
    x_obst[0]+=(-x0[3])*dt
    x_obst[1]+=(x0[3])*dt
    x_obst[2]+=(-x0[4])*dt
    x_obst[3]+=(x0[4])*dt
    #x0=[[x],[y],[z],[v_x],[v_y],[v_z],[roll],[pitch]]
    #cost= np.power((x0[0]-x_ref[0]),2)+np.power((x0[1]-x_ref[1]),2)+np.power((x0[2]-x_ref[2]),2)+np.power((x0[3]-x_ref[3]),2)+np.power((x0[4]-x_ref[4]),2)+np.power((x0[5]-x_ref[5]),2) + cost
    #print(Q_state)
    cost+= cs.sumsqr(Q_state*(x0-x_ref))+cs.sumsqr(Q_hovering*(u0-u_hover))
    #A=x0-x_ref
    #B=Q_state*A
    #print(A[5])
    #print(B[5])
    #obstacle avoidance from 2D lidar
    #cost+= Q_obstacle[0]*cs.fmax(d_safe-x_obst[0]+x0[3]*dt,0) #+x
    #cost+= Q_obstacle[1]*cs.fmax(0.0,d_safe-x_obst[1]-x0[3]*dt) #-x
    #cost+= Q_obstacle[2]*cs.fmax(d_safe-x_obst[2]+x0[4]*dt,0) #+y
    #cost+= Q_obstacle[3]*cs.fmax(d_safe-x_obst[3]-x0[4]*dt,0) #-y
    #print(x0[5])
    if i < N - 1:
        cost+=cs.sumsqr(Q_u*(u0-u[(i+1)*nu:(i+1)*nu+nu]))
    # Constraints
    #c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[0]*(d_safe-x_obst[0]-(-x0[3])*dt))) #+x vx
    #c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[1]*(d_safe-(x_obst[1])-x0[3]*dt))) #-x -vx
    #c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[2]*(d_safe-x_obst[2]-x0[4]*dt))) #+y vy
    #c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[3]*(d_safe-(x_obst[3])-(-x0[4])*dt))) #-y -vy
    c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[0]*(d_safe   -x_obst[0]-    (-x0[1])*dt))) #+x vx
    c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[1]*(d_safe   -x_obst[1]-  (x0[1]*dt)))) #-x -vx
    c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[2]*(d_safe   -x_obst[2]-    (-x0[2]*dt)))) #+y vy
    c= cs.vertcat(c,cs.fmax(0.0,Q_obstacle[3]*(d_safe   -x_obst[3]-   (x0[2])*dt))) #-y -vy
    ###Cylinders
    c = cs.vertcat(c,Q_Obslist[0]*cs.fmax(0, Q_Cylinders[2]**2-(x0[0]-Q_Cylinders[0])**2 - (x0[1]-Q_Cylinders[1])**2))
    c = cs.vertcat(c,Q_Obslist[1]*cs.fmax(0, Q_Cylinders[5]**2-(x0[0]-Q_Cylinders[3])**2 - (x0[1]-Q_Cylinders[4])**2))
    c = cs.vertcat(c,Q_Obslist[2]*cs.fmax(0, Q_Cylinders[8]**2-(x0[0]-Q_Cylinders[6])**2 - (x0[1]-Q_Cylinders[7])**2))
    c = cs.vertcat(c,Q_Obslist[3]*cs.fmax(0, Q_Cylinders[11]**2-(x0[0]-Q_Cylinders[9])**2 - (x0[1]-Q_Cylinders[10])**2))

    

#c = cs.vertcat(cs.fmax(0.0, -u[0] ),
#               cs.fmax(math.pi/2,  u[1] ),
#               cs.fmax(math.pi/2, -u[1] ),
#               cs.fmax(math.pi/2,  u[2] ),
#               cs.fmax(math.pi/2, -u[2] ),
#                 )

tcp_config = og.config.TcpServerConfiguration(bind_port=3301)        
bounds= og.constraints.Rectangle(umin,umax)
problem = og.builder.Problem(u, p, cost) \
.with_penalty_constraints(c)  \
.with_constraints(bounds)
          
          
meta = og.config.OptimizerMeta() \
    .with_version("0.0.0") \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By") \
    .with_optimizer_name("theoptimizer") 

build_config = og.config.BuildConfiguration() \
    .with_build_directory("MPC") \
    .with_build_mode("release") \
.with_tcp_interface_config(tcp_config) \
     .with_build_c_bindings()  

solver_config = og.config.SolverConfiguration() \
    .with_lfbgs_memory(15) \
    .with_tolerance(1e-5) \
    .with_max_inner_iterations(155) \
    .with_max_outer_iterations(3)

builder = og.builder.OpEnOptimizerBuilder(problem,
    metadata=meta,
    build_configuration=build_config,
    solver_configuration=solver_config)
#builder.enable_tcp_interface()
builder.build()

mng = og.tcp.OptimizerTcpManager('MPC/theoptimizer')
mng.start()

pong = mng.ping() # check if the server is alive
print(pong)
x_init= [0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
x_ref = [0.0,1.0,10.0,0.0,0.0,0.0,0.0,0.0];
x_obst = [0.0,3.0,0.0,0.0];
## weights and hovering reference
u_hover=[9.81,0,0]
Q_state=[5.0, 5.0,5.0,5.0,5.0,5.0,1.0,1.0];
Q_hovering=[10.0,10.0,10.0];
Q_u=[20,20,20];
Q_obstacle=[1.0,1.0,1.0,1.0];  # 0: +x 1:-x 2:+y 3: -y
d_safe=[2.0];
Q_Cylinders = [0.0]*4*3
Q_Obslist = [1]*4

x_0=x_init+x_ref + x_obst + d_safe +u_hover+ Q_state+Q_hovering+Q_u+Q_obstacle + Q_Cylinders + Q_Obslist#concatenate lists in Python
solution = mng.call(x_0) # call the solver over TCP
print(solution)
print('u_0:')
u0=solution[u'solution']
print(u0[0:3])


mng.kill()
