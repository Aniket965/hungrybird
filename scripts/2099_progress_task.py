#!/usr/bin/env python

''' 
* Team Id : 2099
* Author List : Aniket Sharma,Ujjwal Upadhyay
* Filename: 2099_progress_task.py
* Theme: eYRC Hungry Bird
* Class: Edrone
* Global Variables: None
'''
# Importing the required libraries

from plutodrone.srv import *
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from std_msgs.msg import Bool
import rospy
import time


class Edrone():
    """
    * ClassName: Edrone
    * Logic: this class runs a ROS-node of name drone_control which holds the 
    * position of e-Drone on the given dummy
    * Example Intialization: e_drone = Edrone()
    """

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z,yaw_value]
        self.drone_position = [0.0, 0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        self.setpoint = [-4.950545846094902, -2.040223581919689, 28.769230769, 0.0]
        self.goal_points = [self.setpoint]
        self.isEmergencyExit = False
        # Declaring a cmd of message type PlutoMsg and initializing values
        self.cmd = PlutoMsg()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.plutoIndex = 0

        # initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        self.Kp = [7, 8, 50, 18.2]
        self.Ki = [0.01, 0.02, 0.0, 0.0]
        self.Kd = [170, 170, 4, 214]
        self.prev_values = [0] * 4
        self.max_values = [1800] * 4
        self.min_values = [1200] * 4
        self.error_sum = [0] * 4
        self.isfirstFlight = True

        # This is the sample time in which we need to run pid.
        self.sample_time = 0.060  # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error, /zero
        self.command_pub = rospy.Publisher(
            '/drone_command', PlutoMsg, queue_size=1)
        self.pub_alt_err = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.pub_pitch_err = rospy.Publisher(
            '/pitch_error', Float64, queue_size=1)
        self.pub_roll_err = rospy.Publisher(
            '/roll_error', Float64, queue_size=1)
        self.pub_yaw_err = rospy.Publisher('/yaw_error', Float64, queue_size=1)
        self.pub_zero = rospy.Publisher('/zero', Float64, queue_size=1)
        self.pub_reqpath = rospy.Publisher('/computepath',Bool,queue_size=1)

  
        # Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch,
        #  /pid_tuning_roll,/input_key
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',
                         PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/drone_yaw', Int16, self.yaw_callback)
        rospy.Subscriber('/input_key', Int16, self.key_callback)
        rospy.Subscriber('/vrep/waypoints', PoseArray, self.set_waypoints)

        # ------------------------------------------------------------------------------------------------------------

        self.arm()  # ARMING THE DRONE

    def disarm(self):
        """
        * Function Name: disarm
        * Input: None
        * Output: None
        * Logic: Disarming condition of the drone
        * Example Call: edrone.disarm()
        """
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def key_callback(self, data):
        """
        * Function Name: key_callback
        * Input: data (keydata)
        * Output: None
        * Logic: arm when data is 20(that is a is pressed), diarm when d pressed
        * Example Call: edrone.key_callback(10)
        """
        if data == 10:
            self.disarm()
            self.isEmergencyExit = True

    def arm(self):
        """
        * Function Name: arm
        * Input: None
        * Output: None
        * Logic: calls diarm first(good practice),then give arming condition of the drone
        * Example Call: edrone.arm()
        """

        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(2)
    def run(self):
        """
        * Function Name: run
        * Input: None
        * Output: None
        * Logic: Request Path, until there are no more waypoints, call pid to hold position of drone at waypoints
        * Example Call: edrone.run()
        """
        self.pub_reqpath.publish(True)
        while self.goal_points:
            print(len(self.goal_points),self.goal_points[0])
            if len(self.goal_points) == 1:
                print("called goals")
                self.pub_reqpath.publish(True)
            while not ((max(max(self.prev_values),-min(self.prev_values)) < 0.5 ) and not self.isfirstFlight):
                self.isfirstFlight = False
                self.pid()
            self.goal_points.pop(0)
            self.isfirstFlight = True
        self.disarm()
        print("disarming")
        # time.sleep(2)


    def set_waypoints(self, wayps):
        """
        * Function Name: set_waypoints
        * Input: waps(Wapoints)
        * Output: None
        * Logic: append given waypoints to goal_points of drone
        * Example Call: edrone.set_waypoints(waps)
        """
        for pose in wayps.poses:
            pos = pose.position
            self.goal_points.append((pos.x, pos.y, pos.z, 0.0))
        print(self.goal_points)

    def whycon_callback(self, msg):
        """
        * Function Name: whycon_callback
        * Input: msg(whycon data)
        * Output: None
        * Logic: The function gets executed each time when /whycon node publishes /whycon/poses
        * it sets first whycon maker detected position to dron position
        * Example Call: edrone.whycon_callback(msg)
        """
        self.drone_position[0] = msg.poses[0].position.x
        # gets firs marker position
        p = msg.poses[0]
        self.drone_position[1:3] = [p.position.y, p.position.z]

  
    def altitude_set_pid(self, alt):
        """
        * Function Name: altitude_set_pid
        * Input: alt(alt data)
        * Output: None
        * Logic: Callback function for /pid_tuning_altitude,
        * This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
        * This function sets kp,ki,kd by multiplying witch factors of Altitude
        * Example Call: edrone.altitude_set_pid(alt)
        """
        self.Kp[2] = alt.Kp * 0.1
        self.Ki[2] = alt.Ki * -0.001
        self.Kd[2] = alt.Kd * 1

    def pitch_set_pid(self, pitch):
        """
        * Function Name: pitch_set_pid
        * Input: pitch(pitch data)
        * Output: None
        * Logic: Callback function for /pid_tuning_pitch,
        * This function gets executed each time when /tune_pid publishes /pid_tuning_pitch
        * This function sets kp,ki,kd by multiplying witch factors of pitch
        * Example Call: edrone.pitch_set_pid(pitch)
        """
        self.Kp[0] = pitch.Kp * 0.1
        self.Ki[0] = pitch.Ki * 0.0001
        self.Kd[0] = pitch.Kd * 1

    def roll_set_pid(self, roll):
        """
        * Function Name: roll_set_pid
        * Input: roll(roll data)
        * Output: None
        * Logic: Callback function for /pid_tuning_roll,
        * This function gets executed each time when /tune_pid publishes /pid_tuning_roll
        * This function sets kp,ki,kd by multiplying witch factors of roll
        * Example Call: edrone.roll_set_pid(roll)
        """
        self.Kp[1] = roll.Kp * 0.1
        self.Ki[1] = roll.Ki * 0.0001
        self.Kd[1] = roll.Kd * 1

    def yaw_set_pid(self, yaw):
        """
        * Function Name: yaw_set_pid
        * Input: yaw(yaw data)
        * Output: None
        * Logic: Callback function for /pid_tuning_yaw,
        * This function gets executed each time when /tune_pid publishes /pid_tuning_yaw
        * This function sets kp,ki,kd by multiplying witch factors of yaw
        * Example Call: edrone.yaw_set_pid(yaw)
        """
        self.Kp[3] = yaw.Kp * 0.1
        self.Ki[3] = yaw.Ki * 0.001
        self.Kd[3] = yaw.Kd * 1
  
    def yaw_callback(self, yaw):
        """
        * Function Name: yaw_callback
        * Input: yaw(yaw data)
        * Output: None
        * Logic: Callback function for /drone_yaw,
        * This function gets executed each time when /drone_service publishes /drone_yaw
        * This function drone current yaw
        * Example Call: edrone.yaw_callback(yaw)
        """
        self.drone_position[3] = yaw.data

    def pid(self):
        """
        * Function Name: pid
        * Input: None
        * Output: None
        * Logic: Calculates Command,error Using pid algorithm,publishes error values and
        * drone command
        * Example Call: edrone.pid()
        """

        # 1 Compute error in each axis

        errors = [dp - setp for dp,
                  setp in zip(self.drone_position, self.goal_points[0])]
        # publishes errors and zero value 
        self.pub_pitch_err.publish(errors[0])
        self.pub_roll_err.publish(errors[1])
        self.pub_alt_err.publish(errors[2])
        self.pub_yaw_err.publish(errors[3])
        self.pub_zero.publish(0.0)
       
        # 2 Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis
        
        der_err = [ce - pr_e for ce, pr_e in zip(errors, self.prev_values)]
        new_error_sum = [ce + es for ce, es in zip(errors, self.error_sum)]
        
        # 3 Calculate the pid output required for each axis.
        out_vals = [sum(p*q for p, q in zip(a, b)) for a, b in zip(zip(errors,
                                                                       new_error_sum, der_err), zip(self.Kp, self.Ki, self.Kd))]
        
        # 4 Reduce or add this computed output value on the avg value ie 1500.
        out_vals = [1500 + val for val in out_vals]

        # 6 Limit the output value and the final command value between the maximum(1800) and minimum(1200)
        def clamp(n, minn, maxn): return max(min(maxn, n), minn)
        out_vals = [clamp(val, self.min_values[i], self.max_values[i])
                    for i, val in enumerate(out_vals)]

        self.cmd.rcPitch = out_vals[0]
        self.cmd.rcRoll = out_vals[1]
        self.cmd.rcThrottle = out_vals[2]
        self.cmd.rcYaw = out_vals[3]

        # 7  Update previous errors
        self.prev_values = errors
        print(self.prev_values)
        # 8 Add error_sum
        self.error_sum = new_error_sum
        
        # 5  Run the pid only at the a sample time. 
        time.sleep(e_drone.sample_time)
        
        # publish command to drone
        self.command_pub.publish(self.cmd)


if __name__ == '__main__':

    e_drone = Edrone()
    time.sleep(2)
    e_drone.run()
    e_drone.disarm()
