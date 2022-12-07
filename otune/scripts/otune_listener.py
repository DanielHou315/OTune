import sys
import os
import time
import math

import rospy
import rosbag

from std_msgs.msg import String, Int16
from geometry_msgs.msg import TwistStamped, Vector3

import otune_optimizer as optimizer


class OTuneListener:
    def __init__(self):

        '''
        Construct an OTuneListener object
        -------------------------------------------------------------------
        The Variables necessary for this program are:

        - Current Time 
            -> for bag name generation
        
        - Tuning Mode

        - Progress
            -> For Recording Start/Stop and Progress Display

        - Vex Velocities
            -> Stores the odom wheels' angular velocity values, in degrees per second
        
        - Vive Velocities
            -> Store the linear and angular velocities of the Vive Tracker (and hence of the robot body)
            -> respective values are in meters per second and radians per second

        -------------------------------------------------------------------
        In this Constructor

        - Check if the default output of bag file directory exists
            -> If not, create one
            -> Then go to that directory

        - Make a bag file and name it with the current time. 

        - Create a ROS Node called otune_listener

        - Create a Subscriber subscribing to VEX Brain messages

        - Create a Subscriber subscribing to Vive_ROS messages
        '''

        # Current time
        self.now_time = time.localtime(time.time())

        # Tuning Mode
        self.tuning_mode = Int16()
        self.tuning_mode.data = -1

        # Progress
        self.progress = 0
        self.vive_updated = False

        # Velocities
        self.vive_velocity = Vector3()
        self.vex_velocity = Vector3()


        # Check Bag Directory
        self.bag_dir = os.path.expanduser('~') +"/Documents/otune/data/"
        if not os.path.isdir(self.bag_dir):
            os.makedirs(self.bag_dir)
        os.chdir(self.bag_dir)

        # Make Bag File
        self.bag_name = "otune_bag_"        \
                        + str(self.now_time.tm_year) + "_" + str(self.now_time.tm_mon)         \
                        + "_" + str(self.now_time.tm_mday) + "_" + str(self.now_time.tm_hour)   \
                        + "_" + str(self.now_time.tm_min) + "_" + str(self.now_time.tm_sec) + ".bag"
        self.data_bag = rosbag.Bag(self.bag_name, 'w')

        # Create Listener ROS Node
        rospy.init_node('otune_listener', anonymous=False)

        # Create VEX Odom Status Subscriber
        rospy.Subscriber('vex/odom_status', String, self.process_odom_status)

        # Create Vive_ROS Subscriber
        # Twist 1 because twist1 outputs the tracker twist when there is no headset connected. 
        rospy.Subscriber('/vive/twist1', TwistStamped, self.cache_tracker_velocity)

    
    def cache_tracker_velocity(self, data):
        '''
        Cache message received from the Vive Tracker
        -------------------------------------------------------------------
        The Message of type geometry_msgs::TwistStamped contains the following information we need: 
        
        - Tracker Linear Velocity                       (TwistStamped[0], float, mps)
            -> This information is computed by taking the hypotenuse of the linear velocity in X and Y direction. 
        
        - Tracker Angular Velocity along the Z-axis     (TwistStamped[1], float, radps)
            -> Since it is 2D motion, we only consider the robot's rotation in the X-Y plane. 

        -------------------------------------------------------------------
        The Message is stored in a Vector3 

        - vector.x = Tracker Linear Velocity
        - vector.y = Tracker Angular Velocity
            -> By doing so we are omitting the timestamp of the message
            -> I suspect that the vive message is going to arrive faster than messages from the brain
                --> so this message is cached and synced to the next brain input. 
        '''
        # Through testing, twist.linear.y is the up-down velocity normal to the tracking plane
        # While these speeds are direcctional, making hypot loses the sign of these velocities
        # Therefore, make your path only go in one direction -- forward 
        self.vive_velocity.x = math.hypot(data.twist.linear.x, data.twist.linear.z)

        # Through testing, twist.angular.y is the rotation along the y axis, pointing up and normal to the plane
        self.vive_velocity.y = data.twist.angular.y
        self.vive_updated = True


    def process_odom_status(self, data):
        '''
        Process message received from the VEX V5 Brain through serial port
        -------------------------------------------------------------------
        The Message of type std_msgs::String contains the following information we need: 
        
        - Tuning Mode       (int)       
            -> this information is only stored once when it is initially transferred
        
        - Progress          (int)       
            -> This information is used by the update_progress process
            -> The range for this number is 0-101
                --> 0 represents that the recording is not in progress
                --> 1-100 represents ceil(actual_progress)
        
        - Wheel 1 Velocity (int, degps)
        
        - Wheel 2 Velocity (int, degps)
        
        - (Potentially) Wheel 3 Velocity (int, degps)
            -> Must accord with tuning_mode: 3 (LRB)
        '''
        # print(data)
        strs = data.data.split('_')
        if len(strs) == 4 or len(strs) == 5:
            # Record Tuning Mode
            if self.tuning_mode.data == -1:
                self.tuning_mode.data = int(strs[0])
                self.data_bag.write('tuning_mode', self.tuning_mode)
                print("Tuning Mode Found: {0}".format(self.tuning_mode.data))
                print('\n')

            self.progress = int(strs[1])

            # Record Velocities
            # transform degps to radps
            self.vex_velocity.x = int(strs[2]) / 180.0 * math.pi
            self.vex_velocity.y = int(strs[3]) / 180.0 * math.pi

            # if three wheel and tuning mode is LRB, then add back wheel velocity 
            if len(strs) == 5 and self.tuning_mode.data == 3:
                self.vex_velocity.z = float(strs[4])

            if (self.vex_velocity.x != 0 or self.vex_velocity.y != 0) and self.progress < 100 and self.vive_updated == True:
                self.data_bag.write('vive_velocity', self.vive_velocity)  
                self.data_bag.write('vex_velocity', self.vex_velocity)
                # Flush Bag
                self.vive_updated == False
                self.data_bag.flush()

            # Update the Progress Bar
            self.update_progress()
        

    def update_progress(self):
        '''
        Update progress bar output to the terminal
        '''
        if self.progress > 100:
            sys.stdout.write("\033[F") # Cursor up one line
            print("100% ["+'#'*20 + ']')
            print("100% Reached")
            rospy.signal_shutdown("Process Completed")
        elif self.progress < 100:
            sys.stdout.write("\033[F") # Cursor up one line
            print("{0}% [".format(self.progress) + '#'*math.floor(self.progress/5) + "-" * (20-math.floor(self.progress/5)) + ']')


    def run(self):
        while not rospy.is_shutdown():
            rospy.rostime.wallsleep(0.005)






if __name__ == '__main__':
    otune = OTuneListener()

    print("Data Recording Started")
    print("Pending Start. Start tuning sequence on your VEX V5 Brain")

    # rospy.spin()
    otune.run()

    print('''Data Recording Completed

    The OTune Bag File from this session is stored at:
    {0}

    To run offline tuning after this program ends, run:

    python3 ~/catkin_ws/src/otune/scripts/otune_optimizer.py {0}

    ---------------------------------------
    Attempting to run optimization script... 
    '''.format(otune.bag_dir+otune.bag_name))

    # Use the OTune_Optimizer Module to complete Optimization
    optimizer.auto_optimization(otune.bag_dir+otune.bag_name)