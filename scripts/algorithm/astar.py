# class Node():
#     """A node class for A* Pathfinding"""

#     def __init__(self, parent=None, position=None):
#         self.parent = parent
#         self.position = position

#         self.g = 0
#         self.h = 0
#         self.f = 0

#     def __eq__(self, other):
#         return self.position == other.position

# class Astar():

#     def __init__(self, config):
#         self.map = config["map"]
#         self.mapinfo = config["mapinfo"]
#         self.maze  = config["maze"]
#         self.start = config["start"]
#         self.end = config["end"]

#     def solve(self):
#         """Returns a list of tuples as a path from the given start to the given end in the given maze"""

#         # Create start and end node
#         start_node = Node(None, self.start)
#         start_node.g = start_node.h = start_node.f = 0
#         end_node = Node(None, self.end)
#         end_node.g = end_node.h = end_node.f = 0

#         # Initialize both open and closed list
#         open_list = []
#         closed_list = []

#         # Add the start node
#         open_list.append(start_node)

#         # Loop until you find the end
#         while len(open_list) > 0:

#             # Get the current node
#             current_node = open_list[0]
#             current_index = 0
#             for index, item in enumerate(open_list):
#                 if item.f < current_node.f:
#                     current_node = item
#                     current_index = index

#             # Pop current off open list, add to closed list
#             open_list.pop(current_index)
#             closed_list.append(current_node)

#             # Found the goal
#             if current_node == end_node:
#                 path = []
#                 current = current_node
#                 while current is not None:
#                     path.append(current.position)
#                     current = current.parent
#                 return path[::-1] # Return reversed path

#             # Generate children
#             children = []
#             for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

#                 # Get node position
#                 node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

#                 # Make sure within range
#                 if node_position[0] > (len(self.maze) - 1) or node_position[0] < 0 or node_position[1] > (len(self.maze[len(self.maze)-1]) -1) or node_position[1] < 0:
#                     continue

#                 # Make sure walkable terrain
#                 if self.maze[node_position[0]][node_position[1]] != 0:
#                     continue

#                 # Create new node
#                 new_node = Node(current_node, node_position)

#                 # Append
#                 children.append(new_node)

#             # Loop through children
#             for child in children:

#                 # Child is on the closed list
#                 for closed_child in closed_list:
#                     if child == closed_child:
#                         continue

#                 # Create the f, g, and h values
#                 child.g = current_node.g + 1
#                 child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
#                 child.f = child.g + child.h

#                 # Child is already in the open list
#                 for open_node in open_list:
#                     if child == open_node and child.g > open_node.g:
#                         continue

#                 # Add the child to the open list
#                 open_list.append(child)



from collections import deque
from algorithm.heapPQ import HeapPQ, HeapNode
class AStar:
    def __init__(self, maze):

        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end
        self.dim = self.width * self.height
        

    def solve(self):

        # visited node
        visited = [False] * self.dim
        # previous node
        previous = [None] * self.dim

        # path count
        count = 0

        completed = False

        inf = float('inf')
        distances = [inf] * self.dim

        unvisited = HeapPQ()

        nodeindex = [None] * self.dim

        endpos = self.end.position

        # To begin, we set the distance to the start to zero (we're there) and add it into the unvisited queue
        distances[self.start.position[0] * self.width + self.start.position[1]] = 0
        startnode = HeapNode(0,self.start)

        nodeindex[self.start.position[0] * self.width + self.start.position[1]] = startnode
        unvisited.insert(startnode)


        while unvisited:
            count += 1

            # remove node that has the minimum cost
            n = unvisited.removeminimum()

            # Current node u, all neighbours will be v
            u = n.value
            upos = u.position

            uposindex = upos[0] * self.width + upos[1]

            if distances[uposindex] == inf:
                break

            # If upos == endpos, we're done!
            if upos == self.end.position:
                completed = True
                break

            for v in u.neighbours:
                if v != None:
                    vpos = v.position
                    vposindex = vpos[0] * self.width + vpos[1]

                    if visited[vposindex] == False:
                        # The extra distance from where we are (upos) to the neighbour (vpos) (manhattan distance)
                        d = abs(vpos[0] - upos[0]) + abs(vpos[1] - upos[1])

                        # New path cost to v is distance to u + extra
                        newdistance = distances[uposindex] + d

                        # A* includes a remaining cost, the f cost. In this case we use manhattan distance to calculate the distance from V to the end. We use manhattan again because A* works well when the g cost and f cost are balanced.
                        remaining = abs(vpos[0] - endpos[0]) + abs(vpos[1] - endpos[1])

                        # If this new distance is the new shortest path to v
                        if newdistance < distances[vposindex]:
                            vnode = nodeindex[vposindex]

                            # v isn't already in the priority queue - add it
                            if vnode == None:
                                vnode = HeapNode(newdistance + remaining, v)
                                unvisited.insert(vnode)
                                nodeindex[vposindex] = vnode
                                distances[vposindex] = newdistance
                                previous[vposindex] = u

                            # v is already in the queue - decrease its key to re-prioritise it
                            else:
                                unvisited.decreasekey(vnode, newdistance + remaining)
                                distances[vposindex] = newdistance
                                previous[vposindex] = u

            visited[uposindex] = True

        # Backtracking
        pathnode = deque()
        current = self.end

        while current is not None:
            nodepos = current.position[0] * self.width + current.position[1]
            pathnode.appendleft(current)
            current = previous[nodepos]

        path = [coord.position for coord in pathnode]

        return path, count, len(path), completed