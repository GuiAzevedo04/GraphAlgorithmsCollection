from Graph import *

is_directed = bool(input("Is the graph directed? [1] Yes | [0] No:\n"))

my_graph = Graph(is_directed)

my_graph.read_graph_string()

print(my_graph.edges)
print(my_graph.vertices)

print(my_graph.edges[2])
print(my_graph.vertices[1])