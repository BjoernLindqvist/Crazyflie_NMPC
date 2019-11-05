 #!/usr/bin/env python
 # license removed for brevity
import opengen as og
import time 
import csv
import os
from State_update import stateupdate
import math
#####Initialize
mng = og.tcp.OptimizerTcpManager('MAVsim/multiple_n_alm')
mng.start() 

#####Set System Parameters
global ustar
N = 40
nMAV = 1
nu = 3
ns = 8
dt = 0.05
ustar = [0.0] * (nMAV*nu*N)
print(len(ustar))
#xref = [1,(0.5*nMAV/2 - 0.25),1.0,0.0,0.0,0.0,0.0,0.0]
#x0 = [1,(0.5*nMAV/2 - 0.25),0.0,0.0,0.0,0.0,0.0,0.0]
xref = [2,-2,1.0,0.0,0.0,0.0,0.0,0.0]
x0 = [2,-2,0.0,0.0,0.0,0.0,0.0,0.0]
#for j in range(1, nMAV):
#    xref = xref + [xref[0], xref[1] - 0.5*j, xref[2], xref[3], xref[4], xref[5], xref[6], xref[7]]
#    x0 = x0 + [x0[0], x0[1] - 0.5*j, x0[2], x0[3], x0[4], x0[5], x0[6], x0[7]]
#xref = [3,-2,0.5,0.0,0.0,0.0,0.0,0.0] + [-3.0,-2,0.5,0.0,0.0,0.0,0.0,0.0] + [-2,3,1.5,0.0,0.0,0.0,0.0,0.0] + [2,3,1.5,0.0,0.0,0.0,0.0,0.0]  
#x0 = [-3,2,1.5,0.0,0.0,0.0,0.0,0.0] + [3,2,1.5,0.0,0.0,0.0,0.0,0.0] + [2,-3,0.5,0.0,0.0,0.0,0.0,0.0] + [-2,-3,0.5,0.0,0.0,0.0,0.0,0.0] 
xnew = x0
global uold
uold = [9.81,0.0,0.0]*(nMAV)
uref = [9.81,0.0,0.0]
obsdata = [0, -2, 1, 0.8]
z0 = x0 + xref + uref + uold + obsdata

#######Set up CSV
csvRow = ['x_ref', 'y_ref', 'z_ref' ,'vx_ref','vy_ref','vz_ref','theta_ref','phi_ref','x', 'y', 'z' ,'vx','vy','vz','theta','phi', 'solvertime']
timestr = time.strftime("%Y%m%d-%H%M%S")
csvfile = "data2.csv"
os.remove(csvfile)
with open(csvfile, "a") as fp:
    wr = csv.writer(fp, dialect='excel')
    wr.writerow(csvRow)




t = 0
steps = 250
while (t < steps):
    start = time.time()
    if t > 50:
        #xref = [-5,(0.5*nMAV/2 - 0.25),1.0,0.0,0.0,0.0,0.0,0.0] #+ [-3,0.25,1.0,0.0,0.0,0.0,0.0,0.0] + [-3,-0.25,1.0,0.0,0.0,0.0,0.0,0.0] + [-3,-0.75,1.0,0.0,0.0,0.0,0.0,0.0] 
        xref = [-2.0, -2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #for j in range(1, nMAV):
            #xref = xref + [x0[0], x0[1] - 0.5*j, x0[2], xref[3], xref[4], xref[5], xref[6], xref[7]]
            #xref = xref + [xref[0], xref[1] - 0.5*j, xref[2], xref[3], xref[4], xref[5], xref[6], xref[7]]
    z0 = x0 + xref + uref + uold + obsdata
    print(len(z0))
    solution = mng.call(z0, initial_guess=ustar, buffer_len=8*4096)

    ustar = solution['solution']
    print(solution)
    #mcv = solution['max_constraint_violation']
    solvertime = solution['solve_time_ms']
    un = ustar[0:nu*nMAV]
    uold = ustar[0:nu*nMAV]
    for j in range(0,nMAV):
        #print(un[nu*j:(nu*j+nu)])
        xnew[ns*j:(ns*j+ns)] = stateupdate(x0[ns*j:(ns*j+ns)],un[nu*j:(nu*j+nu)])
        print(xnew[ns*j:(ns*j+3)])
    x0 = xnew
    csvRow = [xref[0], xref[1],xref[2] ,xref[3],xref[4],xref[5],xref[6],xref[7]]
    ####Write to CSV file
    for j in range(0,nMAV):
    	csvRow = csvRow + [x0[j*ns], x0[j*ns+1], x0[j*ns+2], x0[j*ns+3], x0[j*ns+4], x0[j*ns+5], x0[j*ns+6], x0[j*ns+7]]
   
    csvRow = csvRow + [solvertime] #+ [un[0]]  + [un[1]] + [un[2]] + [un[3]] + [un[4]] + [un[5]]

 #csvRow = [xref[0], xref[1],xref[2] ,xref[3],xref[4],xref[5],xref[6],xref[7],x0[0], x0[1],x0[2] ,x0[3],x0[4],x0[5],x0[6],x0[7],x0[8],x0[9], x0[10],x0[11] ,x0[12],x0[13],x0[14],x0[15],x0[16],x0[17], x0[18],x0[19] ,x0[20],x0[21],x0[22],x0[23], solvertime]
    with open(csvfile, "a") as fp:
        wr = csv.writer(fp, dialect='excel')
        wr.writerow(csvRow)

    #print(x0)
    end = time.time()
    #print(solution['solve_time_ms'])
    t = t + 1
    #time.sleep(0.05-(end-start))
    tot_time = time.time()
    loop_time = tot_time - start
    print(loop_time)
    

mng.kill()
print("Simulation Complete")

 

