 #!/usr/bin/env python
 # license removed for brevity
import rospy
import opengen as og
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time 
from quaternion_to_euler import quaternion_to_euler
import math
mng = og.tcp.OptimizerTcpManager('MAV/hoop')
mng.start() 
xpos = 0
ypos = 0
zpos = 0
qx = 0
qy = 0
qz = 0
qw = 0
vx = 0
vy = 0
vz = 0
C = math.sqrt(9.81) / 0.82
obsdata_x = 0.0
obsdata_y = 0.0
obsdata_z = 0.0
obsdata_r = 0.20
#obsdata = [0]*(3)
ustar = [0.0] * (120)
N = 40
nu = 3
dt = 0.05
x0 = [-2,4,0.0,0.0,0.0,0.0,0.0,0.0]
global uold
uold = [9.81,0.0,0.0]
uref = [9.81,0.0,0.0]
obsdata = [0.0,4.0,1.0,0.125]
xref = [-2.0,4.0,1.0,0.0,0.0,0.0,0.0,0.0]
z0 = x0 + xref + uref + uold + obsdata
def callback(data):
    global xpos, ypos, zpos, qx, qy, qz, qw, vx, vy, vz
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y
    zpos = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z  
    #print([qx, qy, qz, qw])

def callback_obs(data):
    global obsdata_x, obsdata_y, obsdata_z, obsdata_r
    obsdata_x = data.pose.pose.position.x
    obsdata_y = data.pose.pose.position.y
    obsdata_z = data.pose.pose.position.z
    obsdata_r = 0.2
    #print(obsdata_x, obsdata_y, obsdata_z, obsdata_r)

def PANOC():
    pub = rospy.Publisher('crazyflie3/cmd_vel', Twist, queue_size=1)
    rospy.init_node('PANOC', anonymous=True)
    sub = rospy.Subscriber('/crazyflie/vicon/crazyflie3/crazyflie3/odom', Odometry, callback)
    sub_obs = rospy.Subscriber('/crazyflie/vicon/hoop/hoop/odom', Odometry, callback_obs)
    rate = rospy.Rate(20) # 20hz
    uold = [9.81, 0.0, 0.0]
    ustar = [0.0] * (120)
    xref = [-2.0,4.0,1.0,0.0,0.0,0.0,0.0,0.0]
    i = 0
    t = 0
    global integrator
    integrator = 0

    while not rospy.is_shutdown():
        xref = [-2.0,4.0,1.0,0.0,0.0,0.0,0.0,0.0]
        start = time.time()
        if t > 200:
            xref = [1.5,4,1.2,0.0,0.0,0.0,0.0,0.0]
        if t > 400:
            xref = [-2,4,1.2,0.0,0.0,0.0,0.0,0.0]
        if t > 600:
            xref = [1.5,4,1.2,0.0,0.0,0.0,0.0,0.0]
        #if t > 800:
        #    xref = [-2,4,1.2,0.0,0.0,0.0,0.0,0.0]
        Euler = quaternion_to_euler(qx,qy,qz,qw)
        obsdata = [obsdata_x, obsdata_y, obsdata_z, obsdata_r]
        x0 = [xpos, ypos, zpos, vx, vy, vz, Euler[0], Euler[1]]
        z0 = x0 + xref + uref + uold + obsdata
        solution = mng.call(z0, initial_guess=ustar)
        ustar = solution['solution']
        #print(solution['max_constraint_violation'])
        uold = ustar[0:3]
        u_r = math.cos(Euler[2])*(-ustar[1]) + math.sin(Euler[2])*ustar[2]
        u_p = -math.sin(Euler[2]) * (-ustar[1])  + math.cos(Euler[2]) * ustar[2]
        #u_r = -ustar[1]
        #u_p = ustar[2]
        cmd_vel = Twist() 
        integrator = integrator + (xref[2] - x0[2])
        u_t = math.sqrt(ustar[0]) / C + 0.0005*integrator
        print(u_t)
        print(x0[2])
        if t < 40:
            u_t = 0.3
            u_r = 0
            u_p = 0
            integrator = 0
        if t > 800:
            u_t = 0.7
        if t > 830:
            u_t = 0
        cmd_vel.linear.x = u_p
        cmd_vel.linear.y = u_r 
        cmd_vel.linear.z = u_t
        cmd_vel.angular.z = -1 * Euler[2]
        pub.publish(cmd_vel)
        rate.sleep()
        end = time.time()
        #print(obsdata)
        #print(xpos,ypos,zpos)
        #print(u_r, u_p, u_t)
        #print(end - start)
        t = t + 1
 
if __name__ == '__main__':
     try:
         PANOC()
     except rospy.ROSInterruptException:
         pass

