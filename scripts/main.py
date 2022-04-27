#!/usr/bin/env python3
# license removed for brevity
# from TurtlebotDriving import TurtlebotDriving
# import rospy

from algorithm.astar import Astar

maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

config = {
    "map": "map/map.pgm",
    "mapinfo": "map/map.yaml",
    "algorithm": "astar",
    "maze": maze,
    "start": (0,0),
    "end": (9,9)
}

def main():
    algorithm = Astar(config)
    path = algorithm.solve()
    print(path)


    try:
        turtlebot = TurtlebotDriving()
        turtlebot.move(path)
        turtlebot.plot_trajectory()
        turtlebot.relaunch()


    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()



