#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [5.68,-1.91,33.40, 0.0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.goal_points = [self.setpoint]

		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
			
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [11.5,10.4,24.0,-5.20]
		self.Ki = [0,0,0,0]
		self.Kd = [47,48,320,25]	


		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_values = [0] *4
		self.max_values = [1800] * 4
		self.min_values = [1200] * 4
		self.error_sum = [0] * 4
		self.isfirstFlight = True






		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.100 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.pub_alt_err = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pub_pitch_err = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.pub_roll_err = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.pub_yaw_err = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.pub_zero = rospy.Publisher('/zero', Float64, queue_size=1)
		self.pub_reqpath = rospy.Publisher('/computepath',Bool,queue_size=1)
		#-----------------------------------------------------------------------------------------------------------
		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		rospy.Subscriber('/drone_yaw', Float64, self.yaw_callback)
		rospy.Subscriber('/vrep/waypoints', PoseArray, self.set_waypoints)





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
	def run(self):
		self.pub_reqpath.publish(True)
		while self.goal_points:
			if len(self.goal_points) == 1:
				print("called goals")
				self.pub_reqpath.publish(True)
			while not ((max(max(self.prev_values),-min(self.prev_values)) < 0.5) and not self.isfirstFlight):
				self.isfirstFlight = False
				self.pid()
			self.goal_points.pop(0)
			self.isfirstFlight = True
		time.sleep(10)
		self.disarm()
		print("disarming")



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		p = msg.poses[0]
		self.drone_position[1:3] = [p.position.y,p.position.z]
		
		#---------------------------------------------------------------------------------------------------------------





	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp 
		self.Ki[2] = alt.Ki
		self.Kd[2] = alt.Kd 

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------
	def pitch_set_pid(self,pitch):
		self.Kp[0] = pitch.Kp
		self.Ki[0] = pitch.Ki
		self.Kd[0] = pitch.Kd

	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp 
		self.Ki[1] = roll.Ki 
		self.Kd[1] = roll.Kd 
	def yaw_set_pid(self,yaw):
		self.Kp[1] = yaw.Kp 
		self.Ki[1] = yaw.Ki 
		self.Kd[1] = yaw.Kd 

	def yaw_callback(self, yaw):
		self.drone_position[3] = yaw.data
	
	def set_waypoints(self, wayps):
		print("got vals")
		for pose in wayps.poses:
			pos = pose.position
			self.goal_points.append((pos.x, pos.y, pos.z, 0.0))


	#----------------------------------------------------------------------------------------------------------------------
	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
		# 1
		errors = [dp - setp for dp,setp in zip(self.drone_position,self.goal_points[0])]
		self.pub_pitch_err.publish(errors[0])
		self.pub_roll_err.publish(errors[1])
		self.pub_alt_err.publish(errors[2])
		self.pub_yaw_err.publish(errors[3])
		self.pub_zero.publish(0.0)
		# 2		
		der_err = [ce - pr_e for ce, pr_e in zip(errors, self.prev_values)]
		new_error_sum = [ ce + es for ce,es in zip(errors,self.error_sum)]
		# 3
		out_vals = [sum(p*q for p,q in zip(a,b)) for a,b in zip(zip(errors,new_error_sum,der_err), zip(self.Kp, self.Ki, self.Kd))]	
		# 4
		out_vals = [1500 + val for val in out_vals]
		
		# 6
		clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
		out_vals = [clamp(val,self.min_values[i],self.max_values[i]) for i,val in enumerate(out_vals)]
		

		self.cmd.rcPitch = out_vals[0]
		self.cmd.rcRoll = out_vals[1]
		self.cmd.rcThrottle = out_vals[2]
		self.cmd.rcYaw = out_vals[3]
		
		# 7 
		self.prev_values = errors
		
		# 8
		self.error_sum = new_error_sum
		# 5
		time.sleep(e_drone.sample_time)
	#------------------------------------------------------------------------------------------------------------------------


		self.command_pub.publish(self.cmd)




if __name__ == '__main__':

	e_drone = Edrone()
	e_drone.run()

	# while not rospy.is_shutdown():
				
