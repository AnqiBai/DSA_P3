import math
import heapq


class RoutePlanner():
    def __init__(self, M, start, goal):
        self.M = M
        self.start = start
        self.goal = goal
        self.frontier = []
        self.visited = set()
        self.came_from = dict()

    def find_neighbors(self, node):
        return self.M.roads[node]

    def get_distance(self, node1, node2):
        x1, y1 = self.M.intersections[node1]
        x2, y2 = self.M.intersections[node2]
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_h_value(self, node):
        return self.get_distance(node, self.goal)

    def get_next_node(self):
        node_f_value, node = heapq.heappop(self.frontier)
        return node

    def construct_final_path(self, current_node):
        path_found = []

        tNode = current_node
        while tNode != self.start:
            path_found.append(tNode)
            tNode = self.came_from[tNode]

        path_found.append(self.start)
        return path_found[-1::-1]

    def a_star_search(self):
        current_node = self.start
        self.visited.add(current_node)
        cost_so_far = {}
        cost_so_far[self.start] = 0
        heapq.heappush(self.frontier, (0, self.start))
        while self.frontier:
            current_node = self.get_next_node()
            if current_node == self.goal:
                break
            neighbors = self.find_neighbors(current_node)
            for nei in neighbors:
                new_cost = cost_so_far[current_node] + \
                    self.get_distance(current_node, nei)
                if nei not in cost_so_far or new_cost < cost_so_far[nei]:
                    cost_so_far[nei] = new_cost
                    heapq.heappush(self.frontier, (new_cost, nei))
                    self.came_from[nei] = current_node

        return self.construct_final_path(self.goal)


def shortest_path(M, start, goal):
    # M.intersections
    # M.roads
    # start from start
    routeplanner = RoutePlanner(M, start, goal)
    return routeplanner.a_star_search()
