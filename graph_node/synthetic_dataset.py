import pandas as pd
import numpy as np
import os
import networkx as nx

def generate_synthetic_dataset(num_nodes=10, num_features=5, output_dir="data"):
    """
    Generates and saves a synthetic disaster response dataset, ensuring the resulting graph is always connected.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")

    # 1. Generate Node Features
    # Using "lat_long" format as requested
    lats = np.random.uniform(low=12.8, high=13.2, size=num_nodes)
    longs = np.random.uniform(low=77.5, high=77.7, size=num_nodes)
    node_ids = [f"{lat:.4f}_{lon:.4f}" for lat, lon in zip(lats, longs)]
    
    features = np.random.rand(num_nodes, num_features)
    feature_names = [
        'temperature', 
        'distance_from_start', 
        'signs_of_life_score', 
        'human_sound_freq', 
        'structural_integrity'
    ]
    df_features = pd.DataFrame(features, columns=feature_names)
    df_features.insert(0, 'gps', node_ids)
    
    features_path = os.path.join(output_dir, "test_nodes_features.csv")
    df_features.to_csv(features_path, index=False)
    print(f"Node features saved to {features_path}")

    # 2. Generate Adjacency Matrix (and ensure it's connected)
    # Create a random, undirected graph
    adj_matrix = np.random.rand(num_nodes, num_nodes)
    adj_matrix = (adj_matrix + adj_matrix.T) / 2
    np.fill_diagonal(adj_matrix, 0)
    
    # Apply a threshold to make it sparse and binary
    adj_matrix[adj_matrix > 0.7] = 1
    adj_matrix[adj_matrix <= 0.7] = 0
    adj_matrix = adj_matrix.astype(int)

    # Ensure Connectivity
    # Create a graph from the matrix to check it
    G = nx.from_numpy_array(adj_matrix)
    
    if not nx.is_connected(G):
        print("Graph is disconnected. Adding edges to connect components...")
        # Get all the separate "islands" (components)
        components = list(nx.connected_components(G))
        # Bridge the components together
        for i in range(len(components) - 1):
            # Pick one node from the current component
            node1 = list(components[i])[0]
            # Pick one node from the next component
            node2 = list(components[i+1])[0]
            # Add an edge in the graph and the matrix
            G.add_edge(node1, node2)
            adj_matrix[node1, node2] = 1
            adj_matrix[node2, node1] = 1
        print("Graph is now connected.")

    # Create a DataFrame with node IDs as index and columns
    df_adj = pd.DataFrame(adj_matrix, index=node_ids, columns=node_ids)
    
    adj_path = os.path.join(output_dir, "test_adjacency.csv")
    df_adj.to_csv(adj_path)
    print(f"Adjacency matrix saved to {adj_path}")
    print("\nSynthetic dataset created successfully")

if __name__ == "__main__":
    generate_synthetic_dataset(num_nodes=5)