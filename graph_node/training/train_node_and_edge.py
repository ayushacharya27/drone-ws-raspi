import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from gnn import GraphSAGEPredictor, EdgePredictor, generate_all_pairs
import numpy as np
import random


def seed_all(seed: int = 42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def train_node_edge(model_node: GraphSAGEPredictor,
                    model_edge: EdgePredictor,
                    data,
                    node_labels,
                    edge_labels,
                    epochs: int = 200,
                    lr: float = 1e-3,
                    weight_decay: float = 1e-5,
                    device: str = 'cpu'):

    model_node = model_node.to(device)
    model_edge = model_edge.to(device)
    data = data.to(device)
    node_labels = node_labels.to(device)
    edge_labels = edge_labels.to(device)

    optimizer = torch.optim.Adam(list(model_node.parameters()) + list(model_edge.parameters()),
                                 lr=lr, weight_decay=weight_decay)

    node_criterion = nn.MSELoss()
    edge_criterion = nn.BCELoss()

    history = {'train_node': [], 'train_edge': []}

    for epoch in range(epochs):
        model_node.train()
        model_edge.train()
        optimizer.zero_grad()

        # Forward
        node_scores = model_node(data)
        src_emb, dst_emb = generate_all_pairs(model_node(data))  # embeddings from node model
        edge_preds = model_edge(src_emb, dst_emb)

        # Loss
        loss_node = node_criterion(node_scores, node_labels)
        loss_edge = edge_criterion(edge_preds, edge_labels)
        loss = loss_node + loss_edge
        loss.backward()
        optimizer.step()

        history['train_node'].append(loss_node.item())
        history['train_edge'].append(loss_edge.item())

        if epoch % 20 == 0 or epoch == epochs-1:
            print(f"Epoch {epoch:3d} | Node Loss: {loss_node:.4f} | Edge Loss: {loss_edge:.4f}")

    return model_node, model_edge, history
