#!/usr/bin/env python3
# license removed for brevity
# from TurtlebotDriving import TurtlebotDriving
# import rospy

from algorithm.astar import Astar
from algorithm.breadthfirst import BFS
from maze import Maze


# config = {
#     "map": "map/map.pgm",
#     "mapinfo": "map/map.yaml",
#     "algorithm": "astar",
#     "maze": maze,
#     "start": (0,1),
#     "end": (17,18)
# }

def main():


    input = [[1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1],
            [1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1],
            [1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,0,1],
            [1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1],
            [1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0,1],
            [1,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,1],
            [1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1],
            [1,0,0,0,1,0,1,0,0,0,1,0,1,0,1,0,1,0,1],
            [1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1],
            [1,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1],
            [1,0,1,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1],
            [1,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1],
            [1,1,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1],
            [1,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,1,0,1],
            [1,0,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1],
            [1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,1,0,1],
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1]]


    maze = Maze(input)
    algorithm = BFS(maze)
    [path, count, length, completed] = algorithm.solve()

    print(path)
    print(count)
    print(length)
    print(completed)


    # try:
    #     turtlebot = TurtlebotDriving()
    #     turtlebot.move(path)
    #     turtlebot.plot_trajectory()
    #     turtlebot.relaunch()


    # except rospy.ROSInterruptException:
    #     pass


if __name__ == '__main__':
    main()

