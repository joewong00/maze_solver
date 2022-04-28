
from collections import deque
class BFS():
    def __init__(self, maze):

        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end
        

    def solve(self):

        # start node
        queue = deque([self.start])

        # visited node
        visited = [False] * (self.width * self.height)

        # path count
        count = 0

        completed = False

        # visited start node
        
        while queue:
            count += 1
            current = queue.pop()

            visited[current.Position[0] * self.width + current.Position[1]] = True

            if current == self.end:
                completed = True
                break

            # Node out of bound
            for n in current.neighbours:
                if n != None:
                    npos = n.Position[0] * self.width + n.Position[1]
                    if visited[npos] == False:
                        queue.appendleft(n)
                        visited[npos] = True
                        prev[npos] = current

        path = deque()
        current = self.end
        while (current != None):
            path.appendleft(current)
            current = prev[current.Position[0] * self.width + current.Position[1]]


        return [path, [count, len(path), completed]]