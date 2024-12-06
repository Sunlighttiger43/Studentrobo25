from typing import Any, List
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations
import math
import pickle
import sys

# Don't use multiprocessing unless prepared to blow up your computer
import multiprocessing
from graph_utils import distance, create_subgraph

# ==================== DON'T RUN THIS MAIN FUNCTION, YOUR COMPUTER WON'T BE HAPPY ======================
# you've been warned...
if __name__ == "__main__":
    with open(sys.path[0] + '/../maps/matrix.pkl', "rb") as f:
        coordinates = list(pickle.load(f))

    def convert_to_adjacency_list(graph_nx):
        adjacency_list = {}
        for node in graph_nx.nodes:
            adjacency_list[node] = []
            for neighbor in graph_nx.neighbors(node):
                weight = graph_nx.edges[node, neighbor].get('weight', 1)
                adjacency_list[node].append((neighbor, weight))
        return adjacency_list

    distance_threshold = 0.3

    num_processes = multiprocessing.cpu_count()

    pool = multiprocessing.Pool(processes=num_processes)

    chunk_size = len(coordinates) // num_processes
    chunks = [
        (coordinates, distance_threshold, i, i + chunk_size)
        for i in range(0, len(coordinates), chunk_size)
    ]

    subgraphs = pool.starmap(create_subgraph, chunks)

    pool.close()
    pool.join()

    graph = nx.compose_all(subgraphs)

    with open(sys.path[0] + "/../maps/graph.pkl", "wb") as f:
        pickle.dump(graph, f)

    with open(sys.path[0] + "/../maps/graph_adj.pkl", 'wb') as f:
        graph = convert_to_adjacency_list(graph)
        pickle.dump(graph, f)

    # # Draw the graph
    # pos = nx.spring_layout(graph)
    # nx.draw(graph, pos, with_labels=True, node_size=500, node_color='skyblue', font_size=10, font_weight='bold')
    # labels = nx.get_edge_attributes(graph, 'weight')
    # nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)
    # plt.title('Undirected Graph')
    # plt.show()



def pathfinding(graph, start, end) -> List[Any]:
    return nx.shortest_path(graph, start, end, weight="weight")  # type: ignore
