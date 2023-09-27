import heapq

class MaxPriorityQueue:
    def __init__(self):
        self.elements = []
    
    def push(self, waypoint_name, value):
        # Push the waypoint with a negated value into the min-heap
        heapq.heappush(self.elements, (-value, waypoint_name))
    
    def pop_max(self):
        if not self.is_empty():
            # Invert the popped value before returning it
            return heapq.heappop(self.elements)[1]
        else:
            raise IndexError("Priority queue is empty")
    
    def is_empty(self):
        return len(self.elements) == 0

    def get_size(self):
        return len(self.elements)
