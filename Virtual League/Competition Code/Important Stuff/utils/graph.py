from typing import Any, List
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations
import math
import pickle

# Don't use multiprocessing unless prepared to blow up your computer
import multiprocessing
from graph_utils import distance, create_subgraph

# ==================== DON'T RUN THIS MAIN FUNCTION, YOUR COMPUTER WON'T BE HAPPY ======================
# you've been warned...
if __name__ == "__main__":
    with open("maps/matrix.pkl", "rb") as f:
        coordinates = list(pickle.load(f))

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

    with open("maps/graph.pkl", "wb") as f:
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
