import pandas as pd
import numpy as np


def load_node_features(nodes_features_csv_path: str):
    """
    Args: node feature CSV into feature matrix and node list
    Returns:
        nodes: GPS node ID list
        features: numpy array [num_nodes, num_features]
    """
    df = pd.read_csv(nodes_features_csv_path)

    nodes = df['gps'].tolist()

    features = df.drop(columns=['gps']).to_numpy(dtype=np.float32)

    return nodes, features


def load_adjacency_matrix(adj_csv_path: str):
    """
    Args: adjacency matrix from CSV into numpy array
    Returns:
        adj_matrix: numpy array [num_nodes, num_nodes]
        nodes: GPS node ID list
    """
    df = pd.read_csv(adj_csv_path, index_col=0)
    nodes = df.index.tolist()
    adj_matrix = df.to_numpy(dtype=np.int32)

    # Ensure undirected: check symmetry
    if not np.array_equal(adj_matrix, adj_matrix.T):
        raise ValueError("Adjacency matrix is not symmetric (undirected).")

    return nodes, adj_matrix


def normalize_features(features: np.ndarray):
    """
    Min-max normalize each feature column: [0,1]
    """
    min_vals = features.min(axis=0)  # column wise
    max_vals = features.max(axis=0)

    ranges = max_vals - min_vals

    denom = ranges.copy()
    denom[denom == 0] = 1

    normalized = (features - min_vals) / denom

    return normalized
