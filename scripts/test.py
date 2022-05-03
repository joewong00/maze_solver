#!/usr/bin/env python3
# license removed for brevity

import rospy
from TurtlebotDriving import TurtlebotDriving
import math


if __name__ == '__main__':
    try:
        turtlebot = TurtlebotDriving()
        turtlebot.forward(1)
        turtlebot.rotate(math.pi/2)
        turtlebot.forward(1)
        turtlebot.rotate(math.pi)
        turtlebot.forward(1)
        turtlebot.rotate(3*math.pi/2)
        turtlebot.forward(1)
        turtlebot.rotate(2*math.pi)
   
        turtlebot.plot_trajectory()
        turtlebot.relaunch()



    except rospy.ROSInterruptException:
        pass