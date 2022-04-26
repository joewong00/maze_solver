#!/usr/bin/env python3
# license removed for brevity
from TurtlebotDriving import TurtlebotDriving
import rospy

def main():
    try:
        turtlebot = TurtlebotDriving()
        turtlebot.closedWalk(1)
        turtlebot.closedRotate(90)
        turtlebot.closedWalk(1)
        turtlebot.closedRotate(90)
        turtlebot.closedWalk(1)
        turtlebot.closedRotate(90)
        turtlebot.closedWalk(1)

        turtlebot.plot_trajectory()
        turtlebot.relaunch()


    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()