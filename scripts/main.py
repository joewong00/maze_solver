#!/usr/bin/env python3
# license removed for brevity
# import rospy
# from TurtlebotDriving import TurtlebotDriving

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import yaml
import logging
import time

from maze import Maze
from algorithm.astar import AStar
from algorithm.breadthfirst import BFS
from algorithm.depthfirst import DFS
from algorithm.djikstra import Djikstra
from algorithm.wallfollower import WallFollower
from algorithm.randommouse import RandomMouse

config = {
    "map_dir":"map",
    "map_info":"map.yaml",
    "algorithm":"wallfollowing"
}

def main():

    # --------------------------------- Input ---------------------------------

    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

    # Map yaml file
    with open(os.path.join(config["map_dir"], config["map_info"])) as file:
        map_config = yaml.load(file, Loader=yaml.FullLoader)

    # Read image
    input = cv2.imread(os.path.join(config["map_dir"], map_config["image"]), -1)

    # Make wall = 1, path = 0
    input = (input != 254).astype(int)
    
    # Maze input
    plt.imshow(input, cmap='gray')
    plt.title("Maze Map")
    plt.axis("off")
    plt.show() 

    print("Creating Maze...")
    t0 = time.time()
    maze = Maze(input)
    t1 = time.time()

    print("\nNode Count:", maze.nodecount)
    print("Time elapsed:", t1-t0, "\n")

    # --------------------------------- Algorithm ---------------------------------

    if config["algorithm"].casefold() == "bfs":
        name = "Breadth First Search"
        algorithm = BFS(maze)
    
    elif config["algorithm"].casefold() == "dfs":
        name = "Depth First Search"
        algorithm = DFS(maze)

    elif config["algorithm"].casefold() == "djikstra":
        name = "Djikstra's Algorithm"
        algorithm = Djikstra(maze)

    elif config["algorithm"].casefold() == "astar":
        name = "A Star Algorithm"
        algorithm = AStar(maze)

    elif config["algorithm"].casefold() == "wallfollowing":
        name = "Right Wall Following"
        algorithm = WallFollower(maze)

    elif config["algorithm"].casefold() == "randommouse":
        name = "Random Mouse Algorithm"
        algorithm = RandomMouse(maze)

    else:
        raise Exception('Algorithm specified not available (BFS, DFS, Astar, Djikstra, WallFollowing, RandomMouse)')
    
    # --------------------------------- Solve Maze ---------------------------------

    logging.info(f'''Starting Solve:
        Algorithm:         {name}
        Map:               {map_config["image"]}
        Map Resolution     {map_config["resolution"]}
    ''')

    # BFS, DFS, Djikstra, Astar
    if name not in ("Right Wall Following", "Random Mouse Algorithm"):

        t0 = time.time()
        path, count, length, completed = algorithm.solve()
        t1 = time.time()

        if completed:
            print("\nPath found:")
            print(path)
            print("Node explored:", count)
            print("Path length:", length)
        
        else:
            print("\nNo path found")
            
        print("Time elapsed:", t1-t0, "\n")

        # --------------------------------- Output Image ---------------------------------

        size = int(20//map_config["resolution"])
        im = np.zeros((size,size))

        for x,y in path:
            im[x,y] = 1

        plt.imshow(im)
        plt.title("Maze Solution")
        plt.axis("off")
        plt.show()

        # --------------------------------- Move Robot ---------------------------------

        # try:
        #     turtlebot = TurtlebotDriving()
        #     turtlebot.move(path)
        #     turtlebot.plot_trajectory()
        #     turtlebot.relaunch()


        # except rospy.ROSInterruptException:
        #     pass


    # --------------------------------- Automous Solving ---------------------------------
    else:

        print(name)


if __name__ == '__main__':
    main()

