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