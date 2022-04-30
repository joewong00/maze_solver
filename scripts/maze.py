import numpy as np

class Maze:
    class Node:
        def __init__(self, position=None):
            self.position = position

            self.neighbours =  [None, None, None, None]

        def __lt__(self, other):
            return (self.position < other.position) 

        def __gt__(self, other):
            return (self.position > other.position)

        def __le__(self, other):
            return (self < other) or (self == other)

        def __ge__(self, other):
            return (self > other) or (self == other)
        

    def __init__(self, arr):

        maze = np.array(arr)

        self.width = maze.shape[0]
        self.height = maze.shape[1]

        self.start = None
        self.end = None
        nodecount = 0
        left = None
        toprownodes = [None] * self.width

        # Starting node
        for y in range(self.height):
            if maze[0,y] == 0:
                self.start = Maze.Node((0,y))
                toprownodes[y] = self.start
                nodecount += 1

        for x in range(1, self.width-1):

            prev = False
            current = False
            next = maze[x,1] == 0

            for y in range(1, self.height-1):
                
                prev = current
                current = next
                next = maze[x,y+1] == 0

                n = None

                if not current:
                    continue

                if prev:
                    if next:
                        n = Maze.Node((x,y))
                        left.neighbours[1] = n
                        n.neighbours[3] = left
                        left = n

                    else:
                        n = Maze.Node((x,y))
                        left.neighbours[1] = n
                        n.neighbours[3] = left
                        left = None

                else:
                    n = Maze.Node((x,y))
                    left = n

                if n != None:
                    if (maze[x-1,y] == 0):
                        t = toprownodes[y]
                        t.neighbours[2] = n
                        n.neighbours[0] = t

                    if (maze[x+1,y] == 0):
                        toprownodes[y] = n
                    
                    else:
                        toprownodes[y] = None

                    nodecount += 1


        # Ending node
        for y in range(self.height):
            if maze[-1,y] == 0:
                self.end = Maze.Node((self.height-1,y))
                t = toprownodes[y]
                t.neighbours[2] = self.end
                self.end.neighbours[0] = t
                nodecount += 1
                break
                
        
        self.nodecount = nodecount
                
