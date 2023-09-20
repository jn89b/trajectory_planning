import heapq 


"""
Data is as follows:
    (priority, data) -> 
    (max_power, [(x1,y1,z1 ), (x2,y2,z2), ... (xn,yn,zn)])

"""

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

    # def get_size(self):
    #     return heapq.qsize()

# Example usage:
pq = MaxPriorityQueue()

# Push waypoints into the priority queue with their values
# pq.push("Waypoint1", 5)
# pq.push("Waypoint2", 3)
# pq.push("Waypoint3", 7)
# pq.push("Waypoint4", 2)

wp_one = [(1,2,3), (4,5,6)]
wp_two = [(7,8,9), (10,11,12)]
wp_three = [(13,14,15), (16,17,18)]
wp_four = [(19,20,21), (22,23,24)]

pq.push(wp_one, 5)
pq.push(wp_two, 3)
pq.push(wp_three, 7)
pq.push(wp_four, 2)


# Pop waypoints with the highest values
while not pq.is_empty():
    highest_value_waypoint = pq.pop_max()
    print(highest_value_waypoint)
