import networkx as nx
import torch
from torch_geometric.data import Data
import numpy as np


def build_nx_graph(nodes, features, adj_matrix):
    """
    Args:
        nodes: GPS node ID list
        features: numpy array [num_nodes, num_features]
        adj_matrix: numpy array [num_nodes, num_nodes]
    Returns:
        G (nx.Graph): undirected NetworkX graph with node attributes
    """
    G = nx.Graph()

    num_nodes = len(nodes)

    for i in range(num_nodes):
        # Use integer `i` for the node and store the "lat_long" ID as an attribute
        G.add_node(i, id=nodes[i], features=features[i])

    for i in range(num_nodes):
        for j in range(i+1, num_nodes):
            if adj_matrix[i, j] == 1:
                G.add_edge(i, j)
                # G.add_edge(j, i)

    return G


def build_pyg_graph(nodes, features, adj_matrix):
    """
    Args:
        nodes: GPS node ID list
        features: numpy array [num_nodes, num_features]
        adj_matrix: numpy array [num_nodes, num_nodes]
    Returns:
        data (torch_geometric.data.Data): PyTorch Geometric graph Data object
    """
    x = torch.tensor(features, dtype=torch.float)

    # row, col = np.where(np.triu(adj_matrix, k=1) == 1)  # only upper triangle: directed
    row, col = np.where(adj_matrix == 1)  # both directions: (i, j) and (j, i)
    edge_index = np.vstack((row, col))  # shape: COO format [2, num_edges]
    edge_index = torch.tensor(edge_index, dtype=torch.long)

    data = Data(x=x, edge_index=edge_index)

    # Custom attribute node_ids: mapping from index(Data) -> GPS node ID
    data.node_ids = nodes

    return data

# def compute_dijkstra_paths(G, source_node):
#     """
#     Runs Dijkstra on a NetworkX graph with node attribute 'features' (assumed 2nd column = distance_from_start).
#     Args:
#         G (nx.Graph)
#         source_node (str): starting GPS node
#     Returns:
#         dict: shortest path lengths
#         dict: actual shortest paths
#     """
#     # Extract edge weights: here we assume 'distance_from_start' is in features[:,1]
#     # NOTE: Each edge gets weight based on avg of the two connected node distances.
#     for u, v in G.edges():
#         dist_u = G.nodes[u]['features'][1]  # distance_from_start feature
#         dist_v = G.nodes[v]['features'][1]
#         G[u][v]['weight'] = (dist_u + dist_v) / 2

#     lengths, paths = nx.single_source_dijkstra(G, source=source_node, weight='weight')
#     return lengths, paths

def compute_dijkstra_paths(G: nx.Graph, source_node_idx: int):
    """
    Runs Dijkstra on a NetworkX graph.
    Args:
        G (nx.Graph): A graph with integer nodes.
        source_node_idx (int): The integer index of the starting node.
    Returns:
        dict: shortest path lengths
        dict: actual shortest paths
    """
    for u, v in G.edges():
        # This part works correctly as u and v are integers
        dist_u = G.nodes[u]['features'][1]
        dist_v = G.nodes[v]['features'][1]
        G[u][v]['weight'] = (dist_u + dist_v) / 2

    # The `source` uses the integer index
    lengths, paths = nx.single_source_dijkstra(G, source=source_node_idx, weight='weight')
    return lengths, paths
