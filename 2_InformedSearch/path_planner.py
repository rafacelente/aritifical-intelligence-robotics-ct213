from tracemalloc import start

from numpy import Inf
from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        self.node_grid.reset()
        pq = []
        start = self.node_grid.get_node(*start_position)
        start.f = 0
        heapq.heappush(pq, (start.f, start)) # Insertion
        while len(pq):
            f, node = heapq.heappop(pq) # Extraction
            if not node.closed:
                node.closed = True
                if node.get_position() == goal_position:
                    return self.construct_path(self.node_grid.get_node(*goal_position)), self.node_grid.get_node(*goal_position).f
                for successor_pos in self.node_grid.get_successors(*node.get_position()):
                    successor = self.node_grid.get_node(*successor_pos)
                    if not successor.closed:
                        if successor.f > node.f + self.cost_map.get_edge_cost(node.get_position(), successor.get_position()):
                            successor.f = node.f + self.cost_map.get_edge_cost(node.get_position(), successor.get_position())
                            successor.parent = node
                            heapq.heappush(pq, (successor.f, successor))
        return self.construct_path(self.node_grid.get_node(*goal_position)), self.node_grid.get_node(*goal_position).f

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		
        self.node_grid.reset()
        pq = []
        start = self.node_grid.get_node(*start_position)
        start.g = start.distance_to(*goal_position)
        start.f = 0
        heapq.heappush(pq, (start.g, start)) # Insertion
        while len(pq):
            f, node = heapq.heappop(pq) # Extraction
            node_pos = node.get_position()
            if not node.closed:
                node.closed = True
                for successor_pos in self.node_grid.get_successors(*node_pos):
                    successor = self.node_grid.get_node(*successor_pos)
                    if not successor.closed:
                        successor.parent = node
                        successor.g = successor.distance_to(*goal_position)
                        successor.f = node.f + self.cost_map.get_edge_cost(node.get_position(), successor.get_position())
                        if successor_pos == goal_position:
                            return self.construct_path(self.node_grid.get_node(*successor_pos)), self.node_grid.get_node(*successor_pos).f
                        heapq.heappush(pq, (successor.g, successor)) # Insertion
        return self.construct_path(self.node_grid.get_node(*goal_position)), self.node_grid.get_node(*goal_position).f


    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        self.node_grid.reset()
        pq = []
        start = self.node_grid.get_node(*start_position)
        start.g = 0
        start.f = start.distance_to(*goal_position)
        heapq.heappush(pq, (start.f, start)) # Insertion
        while len(pq):
            f, node = heapq.heappop(pq) # Extraction
            if not node.closed:
                node.closed = True
                if node.get_position() == goal_position:
                    return self.construct_path(self.node_grid.get_node(*goal_position)), self.node_grid.get_node(*goal_position).f
                for successor_pos in self.node_grid.get_successors(*node.get_position()):
                    successor = self.node_grid.get_node(*successor_pos)
                    if not successor.closed:
                        cost = self.cost_map.get_edge_cost(node.get_position(), successor.get_position())
                        h = successor.distance_to(*goal_position)
                        if successor.f > node.g + cost + h:
                            successor.g = node.g + cost
                            successor.f = successor.g + h
                            successor.parent = node
                            heapq.heappush(pq, (successor.f, successor))

        return self.construct_path(self.node_grid.get_node(*goal_position)), self.node_grid.get_node(*goal_position).f