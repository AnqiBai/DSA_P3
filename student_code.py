import math
import heapq


class RoutePlanner():
    def __init__(self, M, start, goal):
        self.M = M
        self.start = start
        self.goal = goal
        self.open_list = []
        self.visited = set()
        self.parent_node = dict()
        self.node_h_value = dict()
        self.node_g_value = dict()
        self.node_g_value[start] = 0
        self.node_h_value[start] = 0

    def find_neighbors(self, node):
        return self.M.roads[node]

    def get_distance(self, node1, node2):
        x1, y1 = self.M.intersections[node1]
        x2, y2 = self.M.intersections[node2]
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_h_value(self, node):
        return self.get_distance(node, self.goal)

    def add_neighbors(self, node):
        neighbors = self.find_neighbors(node)
        for nei in neighbors:
            if nei not in self.visited:
                self.parent_node[nei] = node
                nei_h_value = self.get_h_value(nei)
                self.node_h_value[nei] = nei_h_value
                nei_g_value = self.node_g_value[node] + \
                    self.get_distance(nei, node)
                self.node_g_value[nei] = nei_g_value
                nei_f_value = nei_h_value + nei_g_value
                heapq.heappush(self.open_list, (nei_f_value, nei))
                self.visited.add(nei)

    def get_next_node(self):
        node_f_value, node = heapq.heappop(self.open_list)
        return node

    def construct_final_path(self, current_node):
        path_found = []

        tNode = current_node
        while tNode != self.start:
            path_found.append(tNode)
            tNode = self.parent_node[tNode]

        path_found.append(self.start)
        return path_found[-1::-1]

    def a_start_search(self):
        current_node = self.start
        current_f_value = self.node_g_value[current_node] + \
            self.node_h_value[current_node]

        heapq.heappush(self.open_list, (current_f_value, current_node))
        self.visited.add(current_node)
        while current_node != self.goal:
            self.add_neighbors(current_node)
            current_node = self.get_next_node()

        return self.construct_final_path(self.goal)


def shortest_path(M, start, goal):
    # M.intersections
    # M.roads
    # start from start
    routeplanner = RoutePlanner(M, start, goal)
    return routeplanner.a_start_search()
