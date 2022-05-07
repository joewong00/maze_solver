#!/usr/bin/env python3
# license removed for brevity
import rospy
from TurtlebotDriving import TurtlebotDriving
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower():
    def __init__(self, speed, distance_wall):
        self.speed = speed
        self.distance_wall = distance_wall
        rospy.init_node('PythonControl')

        self.rate = rospy.Rate(10)
       
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.sleep(2)

    def update_command_vel(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)


    def scan_callback(self, msg):
        scan_max_value = msg.range_max

        # Each region scans 9 degrees
        self.regions = {
            'N':  min(min(msg.ranges[len(msg.ranges) - 4 : len(msg.ranges) - 1] + msg.ranges[0:5]), scan_max_value),
            'NNW':  min(min(msg.ranges[11:20]), scan_max_value),
            'NW':  min(min(msg.ranges[41:50]), scan_max_value),
            'WNW':  min(min(msg.ranges[64:73]), scan_max_value),
            'W':  min(min(msg.ranges[86:95]), scan_max_value),
            'E':  min(min(msg.ranges[266:275]), scan_max_value),
            'ENE':  min(min(msg.ranges[289:298]), scan_max_value),
            'NE':  min(min(msg.ranges[311:320]), scan_max_value),
            'NNE':  min(min(msg.ranges[341:350]), scan_max_value),
            'frontwide': min(min(msg.ranges[0:30] + msg.ranges[-30:]),10)
        }


    def run(self):

        try:
            bot = TurtlebotDriving()

            print('Starting with values:')
            print('- linear speed: ', str(self.speed))
            print('- distance to wall: ', str(self.distance_wall))
            print('')

            g_start_sim_time = rospy.get_time()
            print('Started at {} seconds (sim time)'.format(g_start_sim_time))

            while self.regions['frontwide'] < 10 and not rospy.is_shutdown():
                self.update_command_vel(self.speed, 0)
                self.rate.sleep()


            print("Maze Solved!")
            bot.plot_trajectory('Wall Following')
            bot.relaunch()
            rospy.spin()

        except rospy.ROSInterruptException:
            pass