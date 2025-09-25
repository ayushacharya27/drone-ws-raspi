import torch
from typing import Tuple
import os
from model.gnn import GraphSAGEPredictor, EdgePredictor


def save_node_model(node_model: GraphSAGEPredictor,
                    path: str = "output/node_predictor_model.pt") -> None:
    """
    Saves the node prediction model with its hyperparameters.
    """
    
    output_dir = os.path.dirname(path)
    # Create the directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    torch.save({
        'state_dict': node_model.state_dict(),
        'in_channels': node_model.in_channels,
        'hidden_channels': node_model.hidden_channels,
        'num_layers': node_model.num_layers,
        'dropout': node_model.dropout,
        'aggr': node_model.aggr
    }, path)
    print(f"Node predictor model saved to {path}")


def load_node_model(path: str = "output/node_predictor_model.pt",
                    device: str = 'cpu') -> GraphSAGEPredictor:
    """
    Loads a node prediction model from disk and reconstructs it.
    """
    ckpt = torch.load(path, map_location=device)
    node_model = GraphSAGEPredictor(
        in_channels=ckpt['in_channels'],
        hidden_channels=ckpt['hidden_channels'],
        num_layers=ckpt['num_layers'],
        dropout=ckpt['dropout'],
        aggr=ckpt.get('aggr', 'max') # .get for backward compatibility
    )
    node_model.load_state_dict(ckpt['state_dict'])
    node_model.to(device)
    node_model.eval()
    print(f"Node predictor model loaded from {path}")
    return node_model


# Future Scope: for Dual Model setup

def save_models(node_model: GraphSAGEPredictor,
                edge_model: EdgePredictor,
                node_path: str = "output/node_model.pt",
                edge_path: str = "output/edge_model.pt") -> None:
    """
    Save node and edge models with their hyperparameters.
    """
    torch.save({
        'state_dict': node_model.state_dict(),
        'in_channels': node_model.in_channels,
        'hidden_channels': node_model.hidden_channels,
        'num_layers': node_model.num_layers,
        'dropout': node_model.dropout,
        'aggr': node_model.aggr
    }, node_path)
    
    torch.save({
        'state_dict': edge_model.state_dict(),
        'node_emb_dim': edge_model.mlp[0].in_features // 2,
        'hidden_dim': edge_model.mlp[0].out_features,
        'dropout': edge_model.mlp[2].p
    }, edge_path)
    
    print(f"Node model saved to {node_path}")
    print(f"Edge model saved to {edge_path}")


def load_models(node_path: str = "output/node_model.pt",
                edge_path: str = "output/edge_model.pt",
                device: str = 'cpu') -> Tuple[GraphSAGEPredictor, EdgePredictor]:
    """
    Load node and edge models from disk and reconstruct them.
    """
    node_ckpt = torch.load(node_path, map_location=device)
    edge_ckpt = torch.load(edge_path, map_location=device)

    node_model = GraphSAGEPredictor(
        in_channels=node_ckpt['in_channels'],
        hidden_channels=node_ckpt['hidden_channels'],
        num_layers=node_ckpt['num_layers'],
        dropout=node_ckpt['dropout'],
        aggr=node_ckpt.get('aggr', 'max')
    )
    node_model.load_state_dict(node_ckpt['state_dict'])
    node_model.to(device)
    node_model.eval()

    edge_model = EdgePredictor(
        node_emb_dim=edge_ckpt['node_emb_dim'],
        hidden_dim=edge_ckpt['hidden_dim'],
        dropout=edge_ckpt['dropout']
    )
    edge_model.load_state_dict(edge_ckpt['state_dict'])
    edge_model.to(device)
    edge_model.eval()

    print(f"Node model loaded from {node_path}")
    print(f"Edge model loaded from {edge_path}")

    return node_model, edge_model
