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
mng = og.tcp.OptimizerTcpManager('MAV/multiplefollower3')
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
xpos2 = 0
ypos2 = 0
zpos2 = 0
qx2 = 0
qy2 = 0
qz2 = 0
qw2 = 0
vx2 = 0
vy2 = 0
vz2 = 0
C1 = math.sqrt(9.81) / 0.83
C2 = math.sqrt(9.81) / 0.84
obsdata_x = 0.0
obsdata_y = 0.0
obsdata_z = 0.0
obsdata_r = 0.125
#obsdata = [0]*(3)
N = 40
nu = 6
dt = 0.05
ustar = [0.0] * (N*nu)
solvetime = 0
x0 = [-2,4,0.0,0.0,0.0,0.0,0.0,0.0]
x02 = [0.5,4,0.0,0.0,0.0,0.0,0.0,0.0]
global uold
uold = [9.81,0.0,0.0]
uold2 = [9.81,0.0,0.0]
uref = [9.81,0.0,0.0]
obsdata = [0.0,4.0,1.0,0.125]
xref = [-2.0,3.0,1.0,0.0,0.0,0.0,0.0,0.0]
xref2 = [0.5,3.0,1.0,0.0,0.0,0.0,0.0,0.0]
z0 = x0 + x02 + xref + xref2 + uref + uold + uold2

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

def callback2(data):
    global xpos2, ypos2, zpos2, qx2, qy2, qz2, qw2, vx2, vy2, vz2
    xpos2 = data.pose.pose.position.x
    ypos2 = data.pose.pose.position.y
    zpos2 = data.pose.pose.position.z
    qx2 = data.pose.pose.orientation.x
    qy2 = data.pose.pose.orientation.y
    qz2 = data.pose.pose.orientation.z
    qw2 = data.pose.pose.orientation.w
    vx2 = data.twist.twist.linear.x
    vy2 = data.twist.twist.linear.y
    vz2 = data.twist.twist.linear.z  


def callback_obs(data):
    global obsdata_x, obsdata_y, obsdata_z, obsdata_r
    obsdata_x = data.pose.pose.position.x
    obsdata_y = data.pose.pose.position.y
    obsdata_z = data.pose.pose.position.z
    obsdata_r = 0.25   
    #print(obsdata_x, obsdata_y, obsdata_z, obsdata_r)

def PANOC():
    pub = rospy.Publisher('crazyflie5/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('crazyflie6/cmd_vel', Twist, queue_size=1)
    rospy.init_node('PANOC', anonymous=True)
    sub = rospy.Subscriber('/crazyflie/vicon/crazyflie5/crazyflie5/odom', Odometry, callback)
    sub2 = rospy.Subscriber('/crazyflie/vicon/crazyflie6/crazyflie6/odom', Odometry, callback2)
    sub_obs = rospy.Subscriber('/crazyflie/vicon/crazyflie2/crazyflie2/odom', Odometry, callback_obs)
    rate = rospy.Rate(20) # 20hz
    uold = [9.81, 0.0, 0.0]
    uold2 = [9.81, 0.0, 0.0]
    ustar = [0.0] * (nu*N)
    #xref = [-0.7,4,0.3,0.0,0.0,0.0,0.0,0.0]
    #xref2 = [0.05,4,1.0,0.0,0.0,0.0,0.0,0.0]
    xref = [-2,3.8,0.0,0.0,0.0,0.0,0.0,0.0]
    xref2 = [0.5,3.8,0.0,0.0,0.0,0.0,0.0,0.0]
    i = 0
    t = 0

    while not rospy.is_shutdown():
        start = time.time()
        xref = [obsdata_x-0.5, obsdata_y+3, obsdata_z, 0, 0, 0, 0, 0]
        xref2 = [obsdata_x+0.5, obsdata_y+3, obsdata_z, 0, 0, 0, 0, 0]
        Euler = quaternion_to_euler(qx,qy,qz,qw)
        Euler2 = quaternion_to_euler(qx2,qy2,qz2,qw2)
        #obsdata = [obsdata_x, obsdata_y, obsdata_z, obsdata_r]
        x0 = [xpos, ypos, zpos, vx, vy, vz, Euler[0], Euler[1]]
        x02 = [xpos2, ypos2, zpos2, vx2, vy2, vz2, Euler2[0], Euler2[1]]
        z0 = x0 + x02 + xref + xref2 + uref + uold + uold2
        solution = mng.call(z0, initial_guess=ustar,buffer_len = 4*4096)
        ustar = solution['solution']
        solvetime = solution['solve_time_ms']
        #print(solvetime)
        uold = ustar[0:3]
        uold2 = ustar[3:6]
        u_r = math.cos(Euler[2])*(-ustar[1]) + math.sin(Euler[2])*ustar[2]
        u_p = -math.sin(Euler[2]) * (-ustar[1])  + math.cos(Euler[2]) * ustar[2]
        u_r2 = math.cos(Euler2[2])*(-ustar[4]) + math.sin(Euler2[2])*ustar[5]
        u_p2 = -math.sin(Euler2[2]) * (-ustar[4])  + math.cos(Euler2[2]) * ustar[5]
        #u_r = -ustar[1]
        #u_p = ustar[2]
        cmd_vel = Twist() 
        cmd_vel2 = Twist()
        u_t = math.sqrt(ustar[0]) / C1
        u_t2 = math.sqrt(ustar[3]) / C2
        if t < 40:
            u_t = 0.3
            u_r = 0
            u_p2 = 0
            u_t2 = 0.3
            u_r2 = 0
            u_p = 0
        if t > 4000:
            u_t = 0.7
            u_t2 = 0.7
        if t > 4030:
            u_t = 0
            u_t2 = 0
        cmd_vel.linear.x = u_p
        cmd_vel.linear.y = u_r 
        cmd_vel.linear.z = u_t
        cmd_vel.angular.z = -1 * Euler[2]
        cmd_vel2.linear.x = u_p2
        cmd_vel2.linear.y = u_r2 
        cmd_vel2.linear.z = u_t2
        cmd_vel2.angular.z = -1 * Euler2[2]
        pub.publish(cmd_vel)
        pub2.publish(cmd_vel2)
        rate.sleep()
        end = time.time()
        #print(obsdata)
        #print(xpos,ypos,zpos)
        #print(u_r, u_p, u_t)
        #print(u_r2, u_p2, u_t2)
        print(end - start)
        t = t + 1
 
if __name__ == '__main__':
     try:
         PANOC()
     except rospy.ROSInterruptException:
         pass

