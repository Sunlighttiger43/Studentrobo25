import networkx as nx
import math

# Function to calculate Euclidean distance between two points
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Function to create an undirected graph based on coordinates within a preset distance
def create_subgraph(coordinates, distance_threshold, start, end):
    subgraph = nx.Graph()
    for i in range(start, end):
        for j in range(i+1, len(coordinates)):
            dist = distance(coordinates[i], coordinates[j])
            if dist <= distance_threshold:
                subgraph.add_edge(coordinates[i], coordinates[j], weight=dist)
    return subgraph