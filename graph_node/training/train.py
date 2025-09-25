import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import os

from model.gnn import GraphSAGEPredictor

def train_node_predictor(model: GraphSAGEPredictor,
                         data,
                         node_labels,
                         epochs: int = 100,
                         lr: float = 1e-3,
                         weight_decay: float = 1e-5,
                         device: str = 'cpu'):
    """
    Trains the GraphSAGEPredictor for node-level regression
    """
    model = model.to(device)
    data = data.to(device)
    node_labels = node_labels.to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    criterion = nn.MSELoss()
    history = {'train_loss': []}

    print("Starting Node Priority Model Training")
    for epoch in range(epochs):
        model.train()
        optimizer.zero_grad()
        
        # Forward pass
        node_scores = model(data)
        
        # Loss calculation
        loss = criterion(node_scores, node_labels)
        loss.backward()
        optimizer.step()
        
        history['train_loss'].append(loss.item())
        
        if epoch % 20 == 0 or epoch == epochs - 1:
            print(f"Epoch {epoch:3d} | Node Prediction Loss: {loss.item():.4f}")
    
    print("Training Complete")
    return model, history


def create_directed_graph_from_scores(original_graph: nx.Graph, node_scores: np.ndarray) -> nx.DiGraph:
    """
    Creates a directed graph where edges point from higher to lower priority nodes
    """
    directed_graph = nx.DiGraph()
    directed_graph.add_nodes_from(original_graph.nodes(data=True))
    
    for u, v in original_graph.edges():
        score_u = node_scores[u]
        score_v = node_scores[v]
        
        # Add a directed edge from the node with the higher score to the one with the lower score
        if score_u > score_v:
            directed_graph.add_edge(u, v)
        elif score_v > score_u:
            directed_graph.add_edge(v, u)
            
    return directed_graph


# def save_and_visualize_graph(graph: nx.DiGraph, node_scores: np.ndarray, output_dir: str = 'output'):
#     """
#     Visualizes the directed graph and saves the plot and adjacency matrix with original "lat_long" labels
#     """
#     if not os.path.exists(output_dir):
#         os.makedirs(output_dir)

#     pos = nx.spring_layout(graph, seed=42)
#     # Create a label mapping from the integer index to the "lat_long" string for plotting
#     node_labels = {node: data['id'] for node, data in graph.nodes(data=True)}
    
#     plt.figure(figsize=(12, 12))
#     nx.draw(graph, pos, labels=node_labels, with_labels=True, node_color=node_scores,
#             cmap=plt.cm.YlOrRd, node_size=1000, font_size=10, font_weight='bold',
#             arrows=True, arrowstyle='->', arrowsize=20)
#     plt.title("Sentry Drone - Directed Rescue Priority Graph (High to Low)")
    
#     plot_path = os.path.join(output_dir, 'final_directed_graph.png')
#     plt.savefig(plot_path)
#     print(f"Directed graph visualization saved to {plot_path}")
#     plt.close()

#     # Get the sorted list of "lat_long" string IDs for the CSV headers
#     node_id_list = [data['id'] for i, data in sorted(graph.nodes(data=True))]
#     # Use pandas to save the adjacency matrix with the correct "lat_long" labels
#     adj_df = nx.to_pandas_adjacency(graph, nodelist=sorted(graph.nodes()))
#     adj_df.index = node_id_list
#     adj_df.columns = node_id_list
    
#     matrix_path = os.path.join(output_dir, 'final_adjacency_matrix.csv')
#     adj_df.to_csv(matrix_path)
#     print(f"Adjacency matrix saved to {matrix_path}")


# def save_and_visualize_graph(graph: nx.DiGraph, node_scores: np.ndarray, output_dir: str = 'output'):
#     """
#     Visualizes the directed graph and saves the plot and adjacency matrix with original "lat_long" labels,
#     using a pictorial style similar to the Dijkstra visualization.
#     """
#     if not os.path.exists(output_dir):
#         os.makedirs(output_dir)

#     pos = nx.spring_layout(graph, seed=42) # Consistent layout
    
#     # Create a label mapping from the integer index to the "lat_long" string for plotting
#     node_labels = {node: data['id'] for node, data in graph.nodes(data=True)}
    
#     plt.figure(figsize=(15, 15)) # increased figure size for consistency

#     # Color Start Node as Green
#     # Find the node with the highest priority score to define it as the "start node"
#     start_node_idx = np.argmax(node_scores)

#     # Draw all nodes first, colored by their priority score
#     nx.draw_networkx_nodes(
#         graph, 
#         pos, 
#         node_color=node_scores, 
#         cmap=plt.cm.YlOrRd, 
#         node_size=800
#     )

#     # Draw the highest-priority "start node" on top in green
#     nx.draw_networkx_nodes(
#         graph, 
#         pos, 
#         nodelist=[start_node_idx], 
#         node_color='lightgreen', 
#         node_size=1000, # Make it slightly larger
#         edgecolors='black' # Add a border to make it pop
#     )

#     # Draw the graph elements
#     # Draw all edges first, with lighter color to match the Dijkstra visualization's base
#     nx.draw_networkx_edges(graph, pos, alpha=0.4, width=1.5, edge_color="gray", arrows=True, arrowstyle='->', arrowsize=15)
    
#     # Draw nodes, colored by score and with labels
#     # Use a consistent node size and color mapping
#     nx.draw_networkx_nodes(graph, pos, node_color=node_scores, cmap=plt.cm.YlOrRd, node_size=800) # Slightly larger nodes
#     nx.draw_networkx_labels(graph, pos, labels=node_labels, font_size=9, font_weight='bold') # Clearer labels

#     plt.title("Sentry Drone - Directed Rescue Priority Graph (High to Low)", fontsize=16) # Consistent title size
#     plt.axis('off') # Hide axes for a cleaner look

#     plot_path = os.path.join(output_dir, 'final_directed_graph.png')
#     plt.savefig(plot_path)
#     print(f"Directed graph visualization saved to {plot_path}")
#     plt.close()

#     # Get the sorted list of "lat_long" string IDs for the CSV headers
#     node_id_list = [data['id'] for i, data in sorted(graph.nodes(data=True))]
#     # Use pandas to save the adjacency matrix with the correct "lat_long" labels
#     adj_df = nx.to_pandas_adjacency(graph, nodelist=sorted(graph.nodes()))
#     adj_df.index = node_id_list
#     adj_df.columns = node_id_list
    
#     matrix_path = os.path.join(output_dir, 'final_adjacency_matrix.csv')
#     adj_df.to_csv(matrix_path)
#     print(f"Adjacency matrix saved to {matrix_path}")


# In training/train.py

def save_and_visualize_graph(graph: nx.DiGraph, node_scores: np.ndarray, output_dir: str = 'output'):
    """
    Visualizes the directed graph, highlighting the highest-priority node in dark red
    with a green border, and saves the plot and adjacency matrix.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    pos = nx.spring_layout(graph, seed=42)
    node_labels = {node: data['id'] for node, data in graph.nodes(data=True)}
    
    plt.figure(figsize=(15, 15))

    # Find the node with the highest priority score
    start_node_idx = np.argmax(node_scores)

    # 1. Draw all edges first
    nx.draw_networkx_edges(
        graph, 
        pos, 
        alpha=0.4, 
        width=1.5, 
        edge_color="gray", 
        arrows=True, 
        arrowstyle='->', 
        arrowsize=15
    )
    
    # 2. Draw all nodes, colored by their priority score
    nx.draw_networkx_nodes(
        graph, 
        pos, 
        node_color=node_scores, 
        cmap=plt.cm.YlOrRd, 
        node_size=800
    )

    # 3. Draw the highest-priority "start node" on top with the new color scheme
    nx.draw_networkx_nodes(
        graph, 
        pos, 
        nodelist=[start_node_idx], 
        node_color='lightgreen',      # Set node color to dark red
        node_size=1000,            # Make it slightly larger
        edgecolors='black',   # Set border color to green
        linewidths=2               # Make the border visible
    )

    # 4. Draw all node labels
    nx.draw_networkx_labels(graph, pos, labels=node_labels, font_size=9, font_weight='bold')

    plt.title("Sentry Drone - Directed Rescue Priority Graph (High to Low)", fontsize=16)
    plt.axis('off')

    plot_path = os.path.join(output_dir, 'final_directed_graph.png')
    plt.savefig(plot_path)
    print(f"Directed graph visualization saved to {plot_path}")
    plt.close()

    # Get the sorted list of "lat_long" string IDs for the CSV headers
    node_id_list = [data['id'] for i, data in sorted(graph.nodes(data=True))]
    # Use pandas to save the adjacency matrix with the correct "lat_long" labels
    adj_df = nx.to_pandas_adjacency(graph, nodelist=sorted(graph.nodes()))
    adj_df.index = node_id_list
    adj_df.columns = node_id_list
    
    matrix_path = os.path.join(output_dir, 'final_adjacency_matrix.csv')
    adj_df.to_csv(matrix_path)
    print(f"Adjacency matrix saved to {matrix_path}")