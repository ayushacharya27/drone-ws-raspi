import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import os
from preprocessing.load import load_node_features, load_adjacency_matrix
from preprocessing.build_graph import build_nx_graph, compute_dijkstra_paths

def visualize_and_save_dijkstra(
    graph: nx.Graph,
    paths: dict,
    source_node_idx: int,
    node_ids: list,
    output_path: str
):
    """
    Visualizes the graph with Dijkstra's shortest paths and saves it to a file.
    """
    # Create the output directory if it doesn't exist
    output_dir = os.path.dirname(output_path)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Define a layout for the nodes
    pos = nx.spring_layout(graph, seed=42)
    
    # Create a dictionary for node labels from index to GPS ID
    labels = {i: node_id for i, node_id in enumerate(node_ids)}
    
    plt.figure(figsize=(15, 15))

    # Draw the full graph with light gray edges
    nx.draw_networkx_edges(graph, pos, alpha=0.3, width=1, edge_color="gray")

    # Draw all nodes
    nx.draw_networkx_nodes(graph, pos, node_color="lightblue", node_size=500)
    
    # Highlight the source node in green
    nx.draw_networkx_nodes(graph, pos, nodelist=[source_node_idx], node_color="lightgreen", node_size=700)
    
    # Identify and draw the edges that are part of the shortest paths
    path_edges = set()
    for path in paths.values():
        for i in range(len(path) - 1):
            path_edges.add(tuple(sorted((path[i], path[i+1]))))
            
    nx.draw_networkx_edges(graph, pos, edgelist=list(path_edges), width=2, edge_color="red")
    
    # Draw the labels
    nx.draw_networkx_labels(graph, pos, labels, font_size=8)
    
    plt.title(f"Dijkstra's Shortest Paths from {node_ids[source_node_idx]}", fontsize=16)
    plt.savefig(output_path)
    plt.close()
    print(f"Dijkstra graph visualization saved to {output_path}")


def run_dijkstra_analysis():
    """
    Loads data, builds a graph, and calculates shortest paths using Dijkstra's algorithm
    """
    NODES_CSV_PATH = "data/nodes_features.csv"
    ADJ_CSV_PATH = "data/adjacency.csv"
    OUTPUT_PNG_PATH = "output/dijkstra_paths_graph.png"
    OUTPUT_CSV_PATH = "output/dijkstra_paths.csv"

    node_ids, features = load_node_features(NODES_CSV_PATH)
    _, adj_matrix = load_adjacency_matrix(ADJ_CSV_PATH)

    nx_graph = build_nx_graph(node_ids, features, adj_matrix)
    
    if nx_graph.number_of_nodes() == 0:
        print("Graph is empty. Cannot run Dijkstra.")
        return

    # Choose a Source Node and Run Dijkstra
    source_node_idx = 0
    source_node_id = node_ids[source_node_idx]
    
    print(f"\nRunning Dijkstra from Source Node: {source_node_id} (Index: {source_node_idx})")
    
    lengths, paths_by_index = compute_dijkstra_paths(nx_graph, source_node_idx)

    # Visualize and Save the Graph
    visualize_and_save_dijkstra(nx_graph, paths_by_index, source_node_idx, node_ids, OUTPUT_PNG_PATH)

    # # Format and Print Results    
    # paths_by_id = {
    #     node_ids[end_idx]: [node_ids[step] for step in path]
    #     for end_idx, path in paths_by_index.items()
    # }

    # print("\nShortest Path Lengths (Costs):")
    # for end_node_idx, length in lengths.items():
    #     end_node_id = node_ids[end_node_idx]
    #     print(f"  To {end_node_id}: {length:.4f}")

    # print("\nShortest Paths:")
    # for end_node_id, path in paths_by_id.items():
    #     path_str = " -> ".join(path)
    #     print(f"  To {end_node_id}: {path_str}")

    
    # print("\nSaving paths to CSV")

    # # Prepare data for the DataFrame
    # results_data = []
    # for end_node_idx, cost in lengths.items():
    #     destination_id = node_ids[end_node_idx]
    #     path_list = paths_by_id.get(destination_id, [])
    #     path_str = " -> ".join(path_list)
    #     results_data.append({
    #         'Destination': destination_id,
    #         'Cost': f"{cost:.4f}",
    #         'Path': path_str
    #     })

    # # Create and save the DataFrame
    # results_df = pd.DataFrame(results_data)
    # results_df.to_csv(OUTPUT_CSV_PATH, index=False)
    # print(f"Dijkstra paths saved to {OUTPUT_CSV_PATH}")

    if not lengths:
        print("No paths found from the source node.")
        return

    # Find the end node (the one with the highest cost)
    end_node_idx = max(lengths, key=lengths.get)
    end_node_id = node_ids[end_node_idx]
    
    # Get the final path to that end node
    final_path_indices = paths_by_index[end_node_idx]
    final_path_ids = [node_ids[i] for i in final_path_indices]

    print(f"\nFinal Path Identified (to farthest node: {end_node_id})")
    path_str = " -> ".join(final_path_ids)
    print(f"Path: {path_str}")
    print(f"Cost: {lengths[end_node_idx]:.4f}")

    # Save the final path to a 1D CSV
    path_df = pd.DataFrame(final_path_ids, columns=['Path_Node_ID'])
    path_df.to_csv(OUTPUT_CSV_PATH, index=True)
    # Saved CSV has an index column starting from 0
    print(f"Final Dijkstra path saved to {OUTPUT_CSV_PATH}")