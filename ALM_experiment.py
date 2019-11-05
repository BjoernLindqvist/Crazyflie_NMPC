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
mng = og.tcp.OptimizerTcpManager('MAVsim/multiple_n_alm')
mng.start() 
C = math.sqrt(9.81) / 0.83
obsdata_x = 0.0
obsdata_y = 0.0
obsdata_z = 0.0
obsdata_r = 0.125
yaw = 0
yaw2 = 0
yaw3 = 0
yaw4 = 0
#obsdata = [0]*(3)
N = 40
nu = 3
ns = 8
nMAV = 1
dt = 0.05
ustar = [0.0] * (N*nu*nMAV)
solvetime = 0
global x0
x0 = [-2,4,0.0,0.0,0.0,0.0,0.0,0.0] * (nMAV)
global uold
uold = [9.81,0.0,0.0] * (nMAV)
uref = [9.81,0.0,0.0]
obsdata = [0.0,0.0,1.0,0.125]
xref = [-2.0,4.0,1.0,0.0,0.0,0.0,0.0,0.0] * (nMAV)
z0 = x0 + xref  + uref + uold 

def callback(data):
    global yaw
    [roll, pitch, yaw] = quaternion_to_euler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    x0[0:ns] = [data.pose.pose.position.x,  data.pose.pose.position.y, data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z, roll, pitch]

#def callback2(data):
#    global yaw2
#    [roll, pitch, yaw2] = quaternion_to_euler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
#    x0[ns:2*ns] = [data.pose.pose.position.x,  data.pose.pose.position.y, data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y, #data.twist.twist.linear.z, roll, pitch]

#def callback3(data):
#    global yaw3
#    [roll, pitch, yaw3] = quaternion_to_euler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
#    x0[2*ns:3*ns] = [data.pose.pose.position.x,  data.pose.pose.position.y, data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y, #data.twist.twist.linear.z, roll, pitch]

#def callback4(data):
#    global yaw4
#    [roll, pitch, yaw4] = quaternion_to_euler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
#    x0[3*ns:4*ns] = [data.pose.pose.position.x,  data.pose.pose.position.y, data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y, #data.twist.twist.linear.z, roll, pitch]


def callback_obs(data):
    global obsdata_x, obsdata_y, obsdata_z, obsdata_r
    obsdata_x = data.pose.pose.position.x
    obsdata_y = data.pose.pose.position.y
    obsdata_z = data.pose.pose.position.z
    obsdata_r = 0.6   

def PANOC():
    pub = rospy.Publisher('crazyflie3/cmd_vel', Twist, queue_size=1)
    #pub2 = rospy.Publisher('crazyflie2/cmd_vel', Twist, queue_size=1)
    #pub3 = rospy.Publisher('crazyflie3/cmd_vel', Twist, queue_size=1)
    #pub4 = rospy.Publisher('crazyflie4/cmd_vel', Twist, queue_size=1)
    rospy.init_node('PANOC', anonymous=True)
    sub = rospy.Subscriber('/crazyflie/vicon/crazyflie3/crazyflie3/odom', Odometry, callback)
    #sub2 = rospy.Subscriber('/crazyflie/vicon/crazyflie2/crazyflie2/odom', Odometry, callback2)
    #sub3 = rospy.Subscriber('/crazyflie/vicon/crazyflie3/crazyflie3/odom', Odometry, callback3)
    #sub4 = rospy.Subscriber('/crazyflie/vicon/crazyflie4/crazyflie4/odom', Odometry, callback4)
    sub_obs = rospy.Subscriber('/crazyflie/vicon/hoop/hoop/odom', Odometry, callback_obs)
    rate = rospy.Rate(20) # 20hz
    uold = [9.81, 0.0, 0.0]*(nMAV)
    ustar = [0.0] * (nu*N*nMAV)
    #xref = [-2.2,3.7,1.8,0.0,0.0,0.0,0.0,0.0]
    #xref2 = [-2,4,0.5,0.0,0.0,0.0,0.0,0.0]
    xref = [-2,5,1.0,0.0,0.0,0.0,0.0,0.0]*(nMAV)
    i = 0
    t = 0
    integrator = 0
    integrator2 = 0
    integrator3 = 0
    integrator4 = 0

    while not rospy.is_shutdown():
        start = time.time()
        obsdata = [obsdata_x, obsdata_y, obsdata_z, obsdata_r]
        xref = [-2,4,1.0,0.0,0.0,0.0,0.0,0.0] #+ [-2,3.7,1.0,0.0,0.0,0.0,0.0,0.0] + [-2,4.3,1.0,0.0,0.0,0.0,0.0,0.0] + [-2,4.9,1.0,0.0,0.0,0.0,0.0,0.0]
        #xref = [obsdata_x-0.5,obsdata_y+3,obsdata_z,0.0,0.0,0.0,0.0,0.0] + [obsdata_x+0.5,obsdata_y+3,obsdata_z,0.0,0.0,0.0,0.0,0.0] + [obsdata_x-0.5,obsdata_y+4,obsdata_z,0.0,0.0,0.0,0.0,0.0] + [obsdata_x+0.5,obsdata_y+4,obsdata_z,0.0,0.0,0.0,0.0,0.0]
        if t > 200:
            xref = [1,4,1.0,0.0,0.0,0.0,0.0,0.0] #+ [0.5,3.7,1.0,0.0,0.0,0.0,0.0,0.0] + [0.5,4.3,1.0,0.0,0.0,0.0,0.0,0.0] + [0.5,4.9,1.0,0.0,0.0,0.0,0.0,0.0]
        if t > 300:
            xref = [1,5,1.0,0.0,0.0,0.0,0.0,0.0]
        if t > 400:
            xref = [-2,3,1.0,0.0,0.0,0.0,0.0,0.0]
        z0 = x0 + xref + uref + uold + obsdata
        solution = mng.call(z0, initial_guess=ustar,buffer_len = 4*4096)
        ustar = solution['solution']
        solvetime = solution['solve_time_ms']
        print(solvetime)
        uold = ustar[0:nu*nMAV]

        u_r = math.cos(yaw)*(-ustar[1]) + math.sin(yaw)*ustar[2]
        u_p = -math.sin(yaw) * (-ustar[1])  + math.cos(yaw) * ustar[2]
        #u_r2 = math.cos(yaw2)*(-ustar[4]) + math.sin(yaw2)*ustar[5]
        #u_p2 = -math.sin(yaw2) * (-ustar[4])  + math.cos(yaw2) * ustar[5]
        #u_r3 = math.cos(yaw3)*(-ustar[7]) + math.sin(yaw3)*ustar[8]
        #u_p3 = -math.sin(yaw3) * (-ustar[7])  + math.cos(yaw3) * ustar[8]
        #u_r4 = math.cos(yaw4)*(-ustar[10]) + math.sin(yaw4)*ustar[11]
        #u_p4 = -math.sin(yaw4) * (-ustar[10])  + math.cos(yaw4) * ustar[11]
        integrator = integrator + (xref[2] - x0[2])
        #integrator2 = integrator2 + (xref[10] - x0[10])
        #integrator3 = integrator3 + (xref[18] - x0[18])
        #integrator4 = integrator4 + (xref[26] - x0[26])

        u_t = math.sqrt(ustar[0]) / C + 0.0005*integrator
        #u_t2 = math.sqrt(ustar[3]) / C + 0.0005*integrator2
        #u_t3 = math.sqrt(ustar[6]) / C + 0.0005*integrator3
        #u_t4 = math.sqrt(ustar[9]) / C + 0.0005*integrator4
        cmd_vel = Twist() 
        cmd_vel2 = Twist()
        cmd_vel3 = Twist()
        cmd_vel4 = Twist()
        if t < 40:
            u_t = 0.3
            u_r = 0
            u_p = 0
            #u_t2 = 0.3
            #u_p2 = 0
            #u_r2 = 0
            #u_t3 = 0.3
            #u_p3 = 0
            #u_r3 = 0
            #u_t4 = 0.3
            #u_p4 = 0
            #u_r4 = 0
            integrator = 0
            #integrator2 = 0
            #integrator3 = 0
            #ntegrator4 = 0
        if t > 600:
            u_t = 0.7
            #u_t2 = 0.7
            #u_t3 = 0.7
            #_t4 = 0.7
        if t > 630:
            u_t = 0
            #u_t2 = 0
            #u_t3 = 0
            #_t4 = 0
        cmd_vel.linear.x = u_p
        cmd_vel.linear.y = u_r 
        cmd_vel.linear.z = u_t
        cmd_vel.angular.z = -0.3 * yaw
        #cmd_vel2.linear.x = u_p2
        #cmd_vel2.linear.y = u_r2 
        #cmd_vel2.linear.z = u_t2
        #cmd_vel2.angular.z = -0.3 * yaw2
        #cmd_vel3.linear.x = u_p3
        #cmd_vel3.linear.y = u_r3 
        #cmd_vel3.linear.z = u_t3
        #cmd_vel3.angular.z = -0.3 * yaw3
        #md_vel4.linear.x = u_p4
        #md_vel4.linear.y = u_r4 
        #cmd_vel4.linear.z = u_t4
        #cmd_vel4.angular.z = -0.3 * yaw4
        pub.publish(cmd_vel)
        #pub2.publish(cmd_vel2)
        #pub3.publish(cmd_vel3)
        #pub4.publish(cmd_vel4)
        rate.sleep()
        end = time.time()
        #print(obsdata)
        #print(xpos,ypos,zpos)
        #print(u_r, u_p, u_t)
        #print(u_r2, u_p2, u_t2)
        #print(end - start)
        t = t + 1
 
if __name__ == '__main__':
     try:
         PANOC()
     except rospy.ROSInterruptException:
         pass

