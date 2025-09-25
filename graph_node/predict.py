import torch
import numpy as np
from preprocessing.load import load_node_features, load_adjacency_matrix
from preprocessing.build_graph import build_nx_graph, build_pyg_graph
from model.model_utils import load_node_model
from training.train import create_directed_graph_from_scores, save_and_visualize_graph

def get_scaler_from_training_data(training_csv_path: str):
    """
    Calculates the min and max values from the training data for normalization
    """
    _, features = load_node_features(training_csv_path)
    min_vals = features.min(axis=0)
    max_vals = features.max(axis=0)
    return min_vals, max_vals

def normalize_test_features(features: np.ndarray, min_vals: np.ndarray, max_vals: np.ndarray):
    """
    Normalizes test data using the scaler from the training data
    """
    ranges = max_vals - min_vals

    # Avoid division by zero if a feature had no range in the training data
    denom = ranges.copy()
    denom[denom == 0] = 1

    normalized = (features - min_vals) / denom

    return normalized

def predict_on_test_data():
    """
    Loads a trained model and runs predictions on new, unseen test data.
    """
    TRAINING_NODES_CSV = "data/nodes_features.csv"
    TEST_NODES_CSV = "data/test_nodes_features.csv"
    TEST_ADJ_CSV = "data/test_adjacency.csv"
    MODEL_PATH = "output/node_predictor_model.pt"
    OUTPUT_DIR = "output_test"

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Running Prediction on Device: {device}")

    print(f"Loading trained model from {MODEL_PATH}...")
    try:
        model = load_node_model(path=MODEL_PATH, device=device)
        model.eval() # set model to evaluation mode
    except FileNotFoundError:
        print(f"Error: Model file not found at {MODEL_PATH}. Please run the training script first.")
        return

    # Load and Preprocess Test Data
    print(f"Loading test data from {TEST_NODES_CSV}...")
    try:
        test_node_ids, test_features = load_node_features(TEST_NODES_CSV)
        _, test_adj_matrix = load_adjacency_matrix(TEST_ADJ_CSV)
    except FileNotFoundError:
        print("Error: Test data files not found. Please create them.")
        return

    # Get scaler from Training data
    print("Calculating scaler from training data...")
    min_vals, max_vals = get_scaler_from_training_data(TRAINING_NODES_CSV)
    
    # Normalize Test data using the training scaler
    print("Normalizing test data...")
    test_features_normalized = normalize_test_features(test_features, min_vals, max_vals)

    # Build Graphs for Test Data
    print("Building graphs for test data...")
    test_nx_graph = build_nx_graph(test_node_ids, test_features, test_adj_matrix)
    test_pyg_graph = build_pyg_graph(test_node_ids, test_features_normalized, test_adj_matrix)

    # Run Prediction
    print("Running model prediction...")
    with torch.no_grad():
        node_scores = model(test_pyg_graph.to(device)).cpu().numpy()

    # Generate and Save Final Output
    print("Generating and saving final rescue graph...")
    final_directed_graph = create_directed_graph_from_scores(test_nx_graph, node_scores)
    
    save_and_visualize_graph(
        final_directed_graph, 
        node_scores, 
        output_dir=OUTPUT_DIR
    )
    print(f"\nPrediction Complete. Results saved in '{OUTPUT_DIR}' folder.")


if __name__ == '__main__':
    predict_on_test_data()