from collections import deque
class BFS():
    def __init__(self, maze):

        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end
        

    def solve(self):

        # start node
        frontier = deque([self.start])

        # visited node
        visited = [False] * (self.width * self.height)

        # previous node
        previous = [None] * (self.width * self.height)

        # path count
        count = 0

        completed = False

        visited[self.start.position[0] * self.width + self.start.position[1]] = True
        
        # BFS 
        while frontier:
            count += 1
            current = frontier.pop()

            if current == self.end:
                completed = True
                break

            # Node out of bound
            for n in current.neighbours:

                # neighbour is not a wall
                if n != None:

                    nodepos = n.position[0] * self.width + n.position[1]

                    # neighbour not visited yet
                    if visited[nodepos] == False:
                        frontier.appendleft(n)
                        visited[nodepos] = True
                        previous[nodepos] = current

        
        # Backtracking
        pathnode = deque()
        current = self.end

        while current is not None:
            nodepos = current.position[0] * self.width + current.position[1]
            pathnode.appendleft(current)
            current = previous[nodepos]

        path = [coord.position for coord in pathnode]

        return path, count, len(path), completed