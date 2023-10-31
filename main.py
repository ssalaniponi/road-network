"""
Python code to find the shortest path between any two districts in 
the road network of the Central Region in Malawi using a weighted 
graph data structure

Author:		Steven Felix Salaniponi
Created:	31.10.2023	
"""

# Heap data structure used to represent a priority queue
import heapq


class Node:
    def __init__(self, name):
        self.name = name
        self.neighbors = {}
        self.visited = False

    def add_neighbor(self, neighbor, weight):
        self.neighbors[neighbor] = weight


class Graph:
    def __init__(self):
        self.nodes = {}

    def add_node(self, node):
        if node.name not in self.nodes:
            self.nodes[node.name] = node

    def add_edge(self, start_node, end_node, weight):
        if start_node.name in self.nodes and end_node.name in self.nodes:
            self.nodes[start_node.name].add_neighbor(end_node.name, weight)
            self.nodes[end_node.name].add_neighbor(start_node.name, weight)

    def shortest_path(self, start_node, end_node):
        # Initialise the distances dictionary setting all nodes to infinity except the start node which is set to zero
        distances = {node: float('inf') for node in self.nodes}
        distances[start_node.name] = 0

        # Initialise the dictionary to store all previous nodes
        previous_nodes = {node: None for node in self.nodes}

        # Create a priority queue to store the nodes and their distances
        priority_queue = [(0, start_node.name)]
        while priority_queue:
            # Initially choose the node with the smallest distance
            current_distance, current_node = heapq.heappop(priority_queue)

            # If current node was marked as visited, skip the iteration
            if self.nodes[current_node].visited:
                continue

            # The destination is reached so backtrack from the destination to the source
            if current_node == end_node.name:
                path = []
                path_cost = 0
                while current_node is not None:
                    path.append('"' + current_node + '"')

                    # Calculate the total distance of the path (path_cost)
                    if previous_nodes[current_node] is not None:
                        path_cost += self.nodes[current_node].neighbors[previous_nodes[current_node]]

                    current_node = previous_nodes[current_node]
                shortest_path = list(reversed(path))
                return shortest_path, path_cost

            # For the current node, examine all its neighbors, get their distances, if the distance is shorter update
            for neighbor, weight in self.nodes[current_node].neighbors.items():
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))

            # Mark the current node as visited
            self.nodes[current_node].visited = True

        return None, 0


# Instantiate the Graph
graph = Graph()

# Instantiate the nodes as districts
mchinji = Node('Mchinji')
kasungu = Node('Kasungu')
dowa = Node('Dowa')
ntchisi = Node('Ntchisi')
nkhotakota = Node('Nkhotakota')
salima = Node('Salima')
ntcheu = Node('Ntcheu')
dedza = Node('Dedza')
lilongwe = Node('Lilongwe')

# Add nodes to the Graph
graph.add_node(mchinji)
graph.add_node(kasungu)
graph.add_node(dowa)
graph.add_node(ntchisi)
graph.add_node(nkhotakota)
graph.add_node(salima)
graph.add_node(ntcheu)
graph.add_node(dedza)
graph.add_node(lilongwe)

# Add roads between the districts as edges with weights (distances)
graph.add_edge(mchinji, kasungu, 141)
graph.add_edge(mchinji, lilongwe, 109)
graph.add_edge(lilongwe, dowa, 55)
graph.add_edge(lilongwe, dedza, 92)
graph.add_edge(kasungu, ntchisi, 66)
graph.add_edge(kasungu, dowa, 117)
graph.add_edge(dowa, ntchisi, 38)
graph.add_edge(dowa, salima, 67)
graph.add_edge(ntchisi, nkhotakota, 66)
graph.add_edge(nkhotakota, salima, 112)
graph.add_edge(salima, dedza, 96)
graph.add_edge(dedza, ntcheu, 74)

# Execute to find the shortest path between the start district and the end district
start_district = lilongwe
end_district = salima
shortest_path, path_cost = graph.shortest_path(start_district, end_district)

print("\n")
if shortest_path:
    print(
        f"Shortest path from {start_district.name} to {end_district.name} -> [{', '.join(shortest_path)}]")
    print(f"The total distance of the path (path_cost): {path_cost}")
else:
    print(
        f"There is no path from {start_district.name} to {end_district.name}")
