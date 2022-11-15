#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
PI = 3.1415926535897
from datetime import datetime



class SimpleNavigationClass:
	def __init__(self):
		
		rospy.init_node('controller')
    

		#rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
		rospy.Subscriber("scan", LaserScan, self.clbk_laser, queue_size = 10)
    
		
		global pub
		pub=rospy.Publisher("cmd_vel", Twist, queue_size = 10)
       
		

        # default values for the lidar variables as a placeholder until the actual sensor values are recieved through from the ros topic
		self.lidar_left = 100
		self.lidar_right = 100
		self.lidar_front = 100
		self.lidar_back = 100
		
		self.lidar_45deg = 100
		self.lidar_135deg = 100
		self.lidar_225deg = 100
		self.lidar_315deg = 100

    # Callback function for the Turtlebots Lidar topic ("/scan")
	def clbk_laser(self, msg):
		
		self.lidar_left = msg.ranges[89]
		self.lidar_right = msg.ranges[269]
		
		self.lidar_front = msg.ranges[0]
		self.lidar_back = msg.ranges[179]
		
		self.lidar_45deg = msg.ranges[44]
		self.lidar_135deg = msg.ranges[134]
		self.lidar_225deg = msg.ranges[224]
		self.lidar_315deg = msg.ranges[314]
	
	

    
	
	
	

    # main loop running the control logic
	def runNavigation(self):
        # Creates a message from type Twist
		vel_msg = Twist()
        # Defines the speed at which the robot will move forward (value between 0 and 1)
		vel_msg.linear.x = 0.3
        # Defines the speed at which the robot will turn around its own axis (value between -1 and 1)
		vel_msg.angular.z = 0 #-1 hogre, 1 venstre

        # Defines the frequency in Hz in which the following loop will run
		r = rospy.Rate(10)
		finished = False
		while not finished:
		
		
		
			def rotateR(self):
				dps = 40 #degrees per second
				ta = 120 #target angle
    				#Converting from angles to radians
				angular_dps = dps*2*PI/360
				relative_angle = ta*2*PI/360
   				#set all velocity to 0 to get a clean rotation
				vel_msg.linear.x=0
				vel_msg.linear.y=0
				vel_msg.linear.z=0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
    		
				#if you want to turn counter clockwise, remove minus in front of "abs"
				vel_msg.angular.z = -abs(angular_dps)
    				
				t0 = rospy.Time.now().to_sec()
				current_angle = 0
    		
				while(current_angle < relative_angle):
					pub.publish(vel_msg)
					t1 = rospy.Time.now().to_sec()
					current_angle = angular_dps*(t1-t0)


    				#stop robot after rotating
				vel_msg.angular.z = 0
				pub.publish(vel_msg)
   
				#rospy.spin()

			def threesixty(self):
				dps = 60 #degrees per second
				ta = 359 #target angle
    				#Converting from angles to radians
				angular_dps = dps*2*PI/360
				relative_angle = ta*2*PI/360
   				#set all velocity to 0 to get a clean rotation
				vel_msg.linear.x=0
				vel_msg.linear.y=0
				vel_msg.linear.z=0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
    		
				#if you want to turn counter clockwise, remove minus in front of "abs"
				vel_msg.angular.z = -abs(angular_dps)
    				
				t0 = rospy.Time.now().to_sec()
				current_angle = 0
    		
				while(current_angle < relative_angle):
					pub.publish(vel_msg)
					t1 = rospy.Time.now().to_sec()
					current_angle = angular_dps*(t1-t0)


    				#stop robot after rotating
				vel_msg.angular.z = 0
				pub.publish(vel_msg)
				
			
			
			
			
			
			outerZone = 0.8
			
			#rotate when meeting flat wall head on
			if self.lidar_front < outerZone and self.lidar_45deg>self.lidar_front and self.lidar_315deg>self.lidar_front:
				rotateR(self)
				#rospy.sleep(1)
				vel_msg.linear.x = 0.3
			
			#follow wall going right, and correct path. if outer corner is met, will make a u-turn
			if self.lidar_45deg>outerZone:
				vel_msg.linear.x = 0.3 
				vel_msg.angular.z =0.5;
			
			elif self.lidar_45deg<outerZone:
				vel_msg.linear.x = 0.3 
				vel_msg.angular.z =-0.5;
				
			
			
			now = datetime.now()
			#print(now.second)
			while now.second % 20 == 0:
				print(now.second)
				threesixty(self)
				break
			
			#rotating when meeting a corner
			#if self.lidar_left>outerZone+0.2 and self.lidar_135deg>outerZone+0.4 and self.lidar_45deg>2.5:
			#	rotateL(self)
			#	vel_msg.linear.x = 0.3
			
				
			#print( self.lidar_225deg, self.lidar_315deg)
			#print(self.lidar_45deg,self.lidar_left, self.lidar_135deg)
			#vel_msg.linear.x = 0 
			#vel_msg.angular.z =-0.0;
			pub.publish(vel_msg)
			
           
            
			

            # sleeps for the time needed to ensure that the loop will be executed with the previously defined frequency
			r.sleep()
			
			

if __name__ == '__main__':
	
		nav = SimpleNavigationClass()
		nav.runNavigation()
