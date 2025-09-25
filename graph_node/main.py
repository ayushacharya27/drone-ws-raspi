import torch
import pandas as pd

# 1. Import all necessary functions from the modules
from synthetic_dataset import generate_synthetic_dataset
from preprocessing.load import load_node_features, load_adjacency_matrix, normalize_features
from preprocessing.build_graph import build_nx_graph, build_pyg_graph
from model.gnn import GraphSAGEPredictor, seed_all
from training.train import train_node_predictor, create_directed_graph_from_scores, save_and_visualize_graph
from model.model_utils import save_node_model
from training.run_dijkstra import run_dijkstra_analysis

def main():
    """
    Main function to run the Sentry Drone GNN pipeline.
    """
    # 1.5. Generate Synthetic Dataset (Temporary)
    generate_synthetic_dataset(num_nodes=10)

    # 2. Setup & Configuration
    seed_all(42)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")

    NODES_CSV_PATH = "data/nodes_features.csv"
    ADJ_CSV_PATH = "data/adjacency.csv"
    MODEL_SAVE_PATH = "output/node_predictor_model.pt"

    # 3. Data Loading and Preprocessing
    print("Loading and Preprocessing Data")
    node_ids, features = load_node_features(NODES_CSV_PATH)
    _, adj_matrix = load_adjacency_matrix(ADJ_CSV_PATH)
    
    # Normalize features for the model
    features_normalized = normalize_features(features)

    # Build graphs: NetworkX for structure/visualization, PyG for training
    nx_graph = build_nx_graph(node_ids, features, adj_matrix)
    pyg_graph = build_pyg_graph(node_ids, features_normalized, adj_matrix)
    
    print(f"Loaded {nx_graph.number_of_nodes()} nodes and {nx_graph.number_of_edges()} edges.")

    # 4. Model Training
    # Prepare labels for training
    # For this example, we use the 'signs_of_life_score' (column index 2) as the target priority
    node_labels = torch.tensor(features_normalized[:, 2], dtype=torch.float)

    # Initialize the node predictor model
    model = GraphSAGEPredictor(
        in_channels=features_normalized.shape[1],
        hidden_channels=64,
        num_layers=2,
        aggr="max"
    )

    # Train the model
    trained_model, history = train_node_predictor(
        model=model,
        data=pyg_graph,
        node_labels=node_labels,
        epochs=100,
        device=device
    )
    
    # 5. Save the Trained Model
    save_node_model(trained_model, path=MODEL_SAVE_PATH)

    # 6. Prediction and Directed Graph Generation
    print("\nGenerating Final Directed Rescue Graph")
    trained_model.eval()
    with torch.no_grad():
        final_node_scores = trained_model(pyg_graph.to(device)).cpu().numpy()
    
    # Create the final directed graph based on predicted scores
    final_directed_graph = create_directed_graph_from_scores(nx_graph, final_node_scores)
    
    # Visualize and save the final outputs (image and adjacency matrix)
    save_and_visualize_graph(final_directed_graph, final_node_scores, output_dir='output')

    # Visualize and save the final graph based on Dijkstra's Shortest Path for Comparison
    run_dijkstra_analysis()
    
    print("\nPipeline Finished Successfully")


if __name__ == '__main__':
    main()
