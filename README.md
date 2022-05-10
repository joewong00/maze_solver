# Maze Solver using ROS Noetic
Solving maze using several state-of-the-art (SOTA) maze-solving approaches: Breadth-first search, Depth-first search, Dijkstra's algorithm, A* search algorithm and Wall-follower algorithm. 

## Environment

- Ubuntu 18.04LTS 64bit
- ROS-Noetic
- Gazebo 11
- Python 3
---
## To Run
1. Clone this repository
```  
git clone https://github.com/joewong00/maze_solver.git
```

2. Create catkin workspace
```  
mkdir -p catkin_ws/src
```

3. Link the respective package to your workspace
```
sudo ln -fs /path_to_this_repo ~/catkin_ws/src
```

4. ROS Setup
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

5. Make main python file executable
```
chmod a+x ~/catkin_ws/src/maze_solver/scripts/main.py
```

6. Launch python file package for different mazes, change the map config in `main.py` accordingly (map1.yaml)
```
roslaunch maze_solver maze1.launch
```

### Demo Video
[Maze-Solving in ROS](https://youtu.be/ByVJ8pI4pGk)

---
## Creating Own Maze
Create and edit the maze in gazebo, save the maze model in path and change accordingly in `world` and `launch` files. To create map, one can use Gazebo plugin like the one in https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin.git or any other mapping methods. If the map is not too large, one can hard code the grid value (1 for wall and 0 for passage), convert into `pgm` and create the map `yaml` file.
