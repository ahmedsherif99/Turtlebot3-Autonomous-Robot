#!/usr/bin/env python
import rospy
import numpy as np                    			#import numpy for trignometric function, arrays... etc
from numpy.lib.function_base import _diff_dispatcher
from numpy.lib.shape_base import _kron_dispatcher

from geometry_msgs.msg import Twist, Point, Quaternion  	#import msg data type "Twist" to be published
from nav_msgs.msg import Odometry        			#import msg data type "odometry" to be subscribed

from tf.transformations import euler_from_quaternion
import math
import cmath
from path import *
from milestone3.srv import *

##Identifiy Global Variables##
#Polar coordinates 
global p
global alpha
global beta
global total_p
global total_alpha
global diff_p
global diff_alpha
global prev_p
global prev_alpha
#Desired positions we require our robot to reach
global x_desired
global y_desired
global theta_desired
#Current positions our robot is at
global X_pos
global Y_pos
global Theta
#Control parameters
global linear_v	
global angular_v	
global kp,ki,kd 
global kpa,kia,kid
global kbeta 

## Parameter Initialize ##
linear_v  = 0
angular_v = 0
kp = 0.11
ki = 0.00012
kd = 0.12
kpa = 0.91
kia = 0.0012
kda = 0.0011
kbeta = 0.42
X_pos = 0
Y_pos = 0
Theta = 0
theta_desired=0

#Callback function with the information we subscribed to (/turtlebot3/odom)
def Callback (Odometry):	

    global X_pos
    global Y_pos
    global Theta	
    X_pos = round(Odometry.pose.pose.position.x,3)    #Get the robot's position in X
    Y_pos = round(Odometry.pose.pose.position.y,3)	#Get the robot's position in Y
    
    ## Take odometry position of robot and use it in quaternion control ##
    q_angles=Odometry.pose.pose.orientation
    e_angles=[]
    e_angles=euler_from_quaternion([Odometry.pose.pose.orientation.x, Odometry.pose.pose.orientation.y,Odometry.pose.pose.orientation.z, Odometry.pose.pose.orientation.w])
    Theta = round(e_angles[2],3)			#Get the robot's orientation around z

def Polar_Coordinates(cell):				#Calculate polar coordinates
    global diff_p
    global prev_p
    global p
    global diff_alpha
    global prev_alpha					
    global alpha
    global beta	
    global x_desired
    global y_desired
    global theta_desired
    global X_pos
    global Y_pos
    global Theta
    x_desired=cell[0]
    y_desired=cell[1]

    x_delta = x_desired - X_pos		#Calculate the Difference in X direction
    y_delta = y_desired - Y_pos		#Calculate the Difference in Y direction
    p,gamma=cmath.polar(complex(x_delta,y_delta))
    alpha = gamma - Theta	#Calculate angle between the local X-direction of the Mobile Robot and p
    beta = -Theta + (theta_desired*(np.pi/180))	#Calculate angle between p and desired global X-direction of the Mobile Robot
    #print ("gamma= ",gamma ," theta= ", Theta, " alpha= ",alpha)
    #print(p,'/',alpha,'/',beta)
    diff_p=p-prev_p
    diff_alpha=alpha-prev_alpha
    prev_p=p
    prev_alpha=alpha

def SetVelocity(v,w):					#Set the velocity of the robot based on the control decision of the controller
    global vel_msg
    global rate
    
    vel_msg.linear.x = v 				#Linear Velocity
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = w 				#Angular Velocity
    pub.publish(vel_msg)				#Publish msg
    
    #ROS Code Publisher
    rate.sleep()					#Sleep with rate

def Control_Law(cell,Goal=0):					#Control function to move the Mobile Robot
    global p		
    global beta		 
    global alpha		
    global linear_v	
    global angular_v	
    global kp,ki,kd
    global kpa,kia,kda
    global kbeta  
    global total_p,total_alpha
    global diff_p,diff_alpha
    
    #Get Polar Coordinates of the robot
    Polar_Coordinates(cell)
    
    #Condition for alpha angle error
    while abs(alpha)>0.01:
        kpa=0.9
        
        #Get Polar Coordinates of the robot
        Polar_Coordinates(cell)
        
        #Calculate angular velocity based on error
        angular_v = kpa * alpha + kia * total_alpha + kda * diff_alpha
        
        #Set the angular velocity variable sent to SetVelocity function
        w = round(angular_v,2)
        print ("alpha= ",alpha , "/ w= ",w)
        
        #Condition for angular velocity with small alpha
        if abs(alpha)<0.1:
            w=min(angular_v,0.05)
        
        #Send w to SetVelocity function    
        SetVelocity(0,w)
        
        #Update alpha
        total_alpha=total_alpha+alpha        
    
    #Stop robot after error condition finish
    SetVelocity(0,0)
    total_alpha=0
    
    #Condition for distance error
    while  p>0.05:
        kpa = 0.4
        
        #Get Polar Coordinates of the robot        
        Polar_Coordinates(cell)
        
        #Calculate Linear velocity based on error
        linear_v = kp * p - ki * total_p -kd * diff_p
        linear_v = max(linear_v,0.15)
        
        #Calculate angular velocity based on error
        angular_v = kpa * alpha + kia * total_alpha + kda * diff_alpha
        
        #Set the Linear and angular velocity variable sent to SetVelocity function
        v = round(linear_v,2) 	#Linear Velocity
        w = round(angular_v,3)	#angular velocity
        print ("p= ",p , "/ v= ",v,"/ w= ",w)
                
        #Condition for linear velocity with small p
        if p <0.2:
            linear_v=min(linear_v,0.05)

	#Send v,w to SetVelocity function
        SetVelocity(v,w)
        
        #Update P and alpha
        total_p=total_p+p
        total_alpha=total_alpha+alpha

    #Stop robot after error condition finish    
    SetVelocity(0,0)
    if Goal==1:
        SetVelocity(0,0)
        while abs(beta) >0.05:
            Polar_Coordinates(cell)
            angular_v = kbeta * beta
            w = round(angular_v,3)
        #    print ("beta= ",beta , "/ w= ",w)
            SetVelocity(0,w)
        SetVelocity(0,0)
        print("Goal Reached")
    
def navigate(m):

    global diff_p
    global prev_p
    global x_desired
    global y_desired
    global theta_desired
    global p		
    global beta		 
    global alpha		
    global total_p,total_alpha
    global diff_alpha,prev_alpha
    theta_desired=0

    if theta_desired>180:
        theta_desired=theta_desired-360	
    elif abs(theta_desired)==0:
        theta_desired=2
    elif abs(theta_desired)==180:
        theta_desired=178

    total_p=0
    diff_p = 0
    prev_p =0
    total_alpha=0
    diff_alpha = 0
    prev_alpha =0
    Goal=0
    for cell in m.solution[1]:
        print(cell)
        if cell == m.solution[1][len(m.solution[1])-1]:
            Goal=1
            Control_Law(cell,Goal)
        else:
            Goal=0
            Control_Law(cell,Goal)



def GO_Confirmation(req):
    global diff_p
    global prev_p
    global x_desired
    global y_desired
    global theta_desired
    global p		
    global beta		 
    global alpha		
    global total_p,total_alpha
    global diff_alpha,prev_alpha
    x_desired=req.a
    y_desired=req.b 
    Goal=req.goal
    if theta_desired>180:
        theta_desired=theta_desired-360	
    elif abs(theta_desired)==0:
        theta_desired=2
    elif abs(theta_desired)==180:
        theta_desired=178
    total_p=0
    diff_p = 0
    prev_p =0
    total_alpha=0
    diff_alpha = 0
    prev_alpha =0

    point=(x_desired,y_desired)
    Control_Law(point,Goal)

    txt = "Point Reached"
    return goResponse(txt)

  


if __name__ == '__main__':
    global rate
    
    #Initialize Node, Publisher and Subscriber Setup
    rospy.init_node('Turtlebot3_Control', anonymous=True) 					#Initialize ROS node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)					#Publish to velocity commands to the turtle topic 
    rate = rospy.Rate(10) 									# rate of publishing msg 10hz
    sub = rospy.Subscriber('odom', Odometry, Callback)	
    s = rospy.Service('go', go, GO_Confirmation)				#Subscribe to the pose topic to get feedback on the position
    vel_msg = Twist()
    SetVelocity(0,0)
    pub.publish(vel_msg)
    
    while not rospy.is_shutdown():
       
        """ map=FindPath()
        print(map.solution[1])
        navigate(map) """
