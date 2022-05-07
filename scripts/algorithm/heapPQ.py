import heapq
class HeapNode:
    # Heap node contains the the node and its cost

    def __init__(self, key, value):
        self.key = key
        self.value = value

class HeapPQ:
    def __init__(self):
        self.pq = []
        self.removed = set()
        self.count = 0

    def __len__(self):
        return self.count

    def insert(self, node):
        """Insert heap node into the heap queue as a tuple."""

        entry = node.key, node.value
        if entry in self.removed:
            self.removed.discard(entry)
        heapq.heappush(self.pq, entry)
        self.count += 1

    def minimum(self):
        """Return the minimum cost heap node."""
        
        priority, item = heapq.heappop(self.pq)
        node = HeapNode(priority, item)
        self.insert(node)
        return node

    def removeminimum(self):
        """Remove the minimum cost heap node from the heap queue."""

        while True:
            (priority, item) = heapq.heappop(self.pq)
            if (priority, item) in self.removed:
                self.removed.discard((priority, item))
            else:
                self.count -= 1
                return HeapNode(priority, item)

    def remove(self, node):
        """Remove the specified heap node from the heap queue."""

        entry = node.key, node.value
        if entry not in self.removed:
            self.removed.add(entry)
            self.count -= 1

    def decreasekey(self, node, new_priority):
        """Substitute the cost of the heap node with a new value."""

        self.remove(node)
        node.key = new_priority
        self.insert(node)