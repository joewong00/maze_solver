#!/usr/bin/env python3
# license removed for brevity
import rospy
from TurtlebotDriving import TurtlebotDriving

class WallFollower():
    def __init__(self):
        pass

    def run(self):

        try:
            bot = TurtlebotDriving()
            
            bot.followRightWall()

            print("Maze Solved!")
            bot.plot_trajectory('Wall Following')
            bot.relaunch()
            rospy.spin()

        except rospy.ROSInterruptException:
            pass