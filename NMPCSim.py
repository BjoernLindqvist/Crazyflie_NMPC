 #!/usr/bin/env python
 # license removed for brevity
import rospy
import opengen as og
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time 
from quaternion_to_euler import quaternion_to_euler
from State_update import stateupdate
import math
mng = og.tcp.OptimizerTcpManager('MAV/sim')
mng.start() 
global ustar
ustar = [0.0] * (120)
N = 40
nu = 3
dt = 0.05
xref = [-2,1.0,1.0,0.0,0.0,0.0,0.0,0.0]
x0 = [0.0]*8
global uold
uold = [9.81,0.0,0.0]
uref = [9.81,0.0,0.0]
obsdata = [0.0,0.0,1.0,0.125]
z0 = x0 + xref + uref + uold + obsdata
print(z0)

def PANOC():
    pub = rospy.Publisher('odom', Odometry, queue_size=1)
    pub2 = rospy.Publisher('data',Odometry, queue_size=1)
    pub3 = rospy.Publisher('input',Twist, queue_size=1)
    rospy.init_node('PANOC', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    uold = [9.81, 0.0, 0.0]
    ustar = [0.0] * (120)
    xref = [-2,1.0,1.0,0.0,0.0,0.0,0.0,0.0]
    #obsdata = [0.0,0.0,1.0,0.125]
    global x0
    x0 = [-2,1.0,1.0,1.0,0.0,0.0,0.0,0.0]
    i = 0
    t = 0
    print(t)

    while not rospy.is_shutdown():
        start = time.time()
        if (t >= 0) & (t < 50):
            x0 = [-2,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            xref = [-2,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            obsdata = [0,5,1.5,0.8]
        if (t > 50) & (t < 100):
            xref = [-2,0.0,1.0,0.0,0.0,0.0,0.0,0.0]
            obsdata = [0,5,1.5,0.8]
        if t > 100:
            xref = [2,0,1.0,0.0,0.0,0.0,0.0,0.0]
            obsdata = [0,5,1.5,0.8]
        if t > 300:
            xref = [5,0,1.0,0.0,0.0,0.0,0.0,0.0]
            obsdata = [3.5,1,3,0.8]
        z0 = x0 + xref + uref + uold + obsdata
        solution = mng.call(z0, initial_guess=ustar, buffer_len=1096*4)

        ustar = solution['solution']
        mcv = solution['max_constraint_violation']
        un = ustar[0:3]
        uold = ustar[0:3]


        solverdata = Odometry()
        solverdata.pose.pose.position.x = solution['solve_time_ms']
        solverdata.pose.pose.position.y = solution['max_constraint_violation']
        solverdata.twist.twist.linear.x = solution['num_inner_iterations']
        solverdata.twist.twist.linear.y = solution['num_outer_iterations']

        inputs = Twist()
        inputs.linear.x = un[1]
        inputs.linear.y = un[2]
        inputs.linear.z = un[0]
        inputs.angular.x = xref[0]
        inputs.angular.y = xref[1]
        inputs.angular.z = xref[2]

        xnew = stateupdate(x0,un)
        x0 = xnew
        odom = Odometry()
        odom.pose.pose.position.x = x0[0]
        odom.pose.pose.position.y = x0[1]
        odom.pose.pose.position.z = x0[2]
        odom.twist.twist.linear.x = x0[3]
        odom.twist.twist.linear.y = x0[4]
        odom.twist.twist.linear.z = x0[5]
        odom.pose.pose.orientation.x = x0[6]
        odom.pose.pose.orientation.y = x0[7]


        pub.publish(odom)
        pub2.publish(solverdata)
        pub3.publish(inputs)
        rate.sleep()
        end = time.time()
        print(t)
        print(x0)
        t = t + 1

if __name__ == '__main__':
     try:
         PANOC()
     except rospy.ROSInterruptException:
         pass
 

