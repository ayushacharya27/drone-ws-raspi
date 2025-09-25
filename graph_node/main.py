from preprocessing.load import load_node_features, load_adjacency_matrix, normalize_features
from preprocessing.build_graph import build_nx_graph, build_pyg_graph, compute_dijkstra_paths

# Load CSVs
nodes, features = load_node_features("data/nodes_features.csv")
_, adj_matrix = load_adjacency_matrix("data/adjacency.csv")

# Normalize features
features = normalize_features(features)

# Build Graphs
G = build_nx_graph(nodes, features, adj_matrix)
pyg_graph = build_pyg_graph(nodes, features, adj_matrix)
print("NetworkX graph:", G.number_of_nodes(), "nodes,", G.number_of_edges(), "edges")
print("PyG graph:", pyg_graph)

# Run baseline Dijkstra (first node as source)
source = nodes[0]
lengths, paths = compute_dijkstra_paths(G, source)
print("Shortest path lengths:", lengths)
print("Paths:", paths)