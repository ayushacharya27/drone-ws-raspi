"""
References:
- GraphSAGE paper: https://arxiv.org/abs/1706.02216
- PyTorch Geometric GraphSAGE docs: https://pytorch-geometric.readthedocs.io/en/latest/generated/torch_geometric.nn.conv.SAGEConv.html
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import SAGEConv
from torch_geometric.data import Data
import numpy as np
import random


def seed_all(seed: int = 42) -> None:
    """
    Set all random seeds for reproducibility
    """
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


class GraphSAGEPredictor(nn.Module):
    """
    GraphSAGE-Pool based model for node-level rescue priority prediction,
    Produces a scalar score in [0, 1] for each node, indicating urgency of rescue based on node sensor features.
    Args:
        in_channels (int): number of input features per node
        hidden_channels (int): hidden dimension size (default: 64)
        num_layers (int): number of GraphSAGE convolution layers (default: 2)
        dropout (float): dropout probability (default: 0.5)
        aggr (str): aggregation method for SAGEConv.
                    options: 'max' (GraphSAGE-Pool), 'mean', 'add'
                    default: 'max'
    """

    def __init__(self,
                 in_channels: int,
                 hidden_channels: int = 64,
                 num_layers: int = 2,
                 dropout: float = 0.5,
                 aggr: str = "max"):
        super(GraphSAGEPredictor, self).__init__()

        self.in_channels = in_channels
        self.hidden_channels = hidden_channels
        self.num_layers = num_layers
        self.dropout = dropout
        self.aggr = aggr

        # GraphSAGE convolution layers
        self.convs = nn.ModuleList()
        self.norms = nn.ModuleList()

        for i in range(num_layers):
            if i == 0:
                self.convs.append(SAGEConv(in_channels, hidden_channels, aggr=aggr))
            else:
                self.convs.append(SAGEConv(hidden_channels, hidden_channels, aggr=aggr))

            # Add LayerNorm after each conv
            self.norms.append(nn.LayerNorm(hidden_channels))

        # Final prediction MLP head
        self.mlp = nn.Sequential(
            nn.Linear(hidden_channels, hidden_channels // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_channels // 2, 1)  # single scalar output (priority score)
        )

    def forward(self, data):
        """
        Args:
            data: either a torch_geometric.data.Data object
                  OR tuple (x, edge_index)
                  - x: node features [num_nodes, in_channels]
                  - edge_index: COO edges [2, num_edges]
        Returns:
            torch.Tensor: rescue priority scores [num_nodes] (each in [0, 1])
        """
        if isinstance(data, Data):
            x, edge_index = data.x, data.edge_index
        else:
            x, edge_index = data

        # GraphSAGE + normalization + ReLU + dropout
        for i in range(self.num_layers):
            x = self.convs[i](x, edge_index)
            x = self.norms[i](x)
            x = F.relu(x)
            if i < self.num_layers - 1:  # no dropout after last conv
                x = F.dropout(x, p=self.dropout, training=self.training)

        # MLP head
        x = self.mlp(x)  # [num_nodes, 1]
        x = torch.sigmoid(x)  # constrain to [0,1]
        return x.squeeze(-1)  # [num_nodes]


# Future Scope

class EdgePredictor(nn.Module):
    """
    Predicts directed edges between node embeddings.
    Input: node embeddings from GraphSAGE
    Output: edge probability (0-1) for each directed pair (i->j)
    """
    def __init__(self, node_emb_dim: int, hidden_dim: int = 64, dropout: float = 0.3):
        super(EdgePredictor, self).__init__()
        self.mlp = nn.Sequential(
            nn.Linear(node_emb_dim * 2, hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, src_emb: torch.Tensor, dst_emb: torch.Tensor):
        edge_input = torch.cat([src_emb, dst_emb], dim=-1)
        return torch.sigmoid(self.mlp(edge_input)).squeeze(-1)


def generate_all_pairs(node_embeddings: torch.Tensor):
    """
    Generate all directed (i,j) pairs (i != j)
    Returns:
        src_emb: [num_pairs, emb_dim]
        dst_emb: [num_pairs, emb_dim]
    """
    num_nodes = node_embeddings.shape[0]
    src_idx, dst_idx = [], []
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                src_idx.append(i)
                dst_idx.append(j)
    src_emb = node_embeddings[torch.tensor(src_idx, device=node_embeddings.device)]
    dst_emb = node_embeddings[torch.tensor(dst_idx, device=node_embeddings.device)]
    return src_emb, dst_emb
