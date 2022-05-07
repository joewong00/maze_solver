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

        # Distance to the start node is 0, add to queue
        distances[self.start.position[0] * self.width + self.start.position[1]] = 0
        startnode = HeapNode(0,self.start)

        nodeindex[self.start.position[0] * self.width + self.start.position[1]] = startnode
        unvisited.insert(startnode)


        while unvisited:
            count += 1

            # remove node that has the minimum cost
            heapnode = unvisited.removeminimum()

            # u is current node, v is all its neighbours
            u = heapnode.value
            upos = u.position

            uposindex = upos[0] * self.width + upos[1]

            if distances[uposindex] == inf:
                break

            # Reach the ending node
            if upos == self.end.position:
                completed = True
                break

            for v in u.neighbours:
                if v != None:
                    vpos = v.position
                    vposindex = vpos[0] * self.width + vpos[1]

                    if visited[vposindex] == False:
                        # The extra distance from where we are (upos) to the neighbour (vpos) (manhattan distance)
                        distance = abs(vpos[0] - upos[0]) + abs(vpos[1] - upos[1])

                        # g(n): path cost to v from the start node is distance to u + extra
                        g = distances[uposindex] + distance

                        # h(n): the path cost to v from the ending node
                        h = abs(vpos[0] - endpos[0]) + abs(vpos[1] - endpos[1])

                        # If this new distance is the new shortest path to v
                        if g < distances[vposindex]:
                            vnode = nodeindex[vposindex]

                            # Add v into the queue if not inside
                            if vnode == None:
                                vnode = HeapNode(g + h, v)
                                unvisited.insert(vnode)
                                nodeindex[vposindex] = vnode
                                distances[vposindex] = g
                                previous[vposindex] = u

                            # Decrease v's value if it is in the queue
                            else:
                                unvisited.decreasekey(vnode, g + h)
                                distances[vposindex] = g
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