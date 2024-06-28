from Graph import *

myGraph = Graph()
stringDoGrafo = myGraph.readtxt()

myGraph.readGraphString(stringDoGrafo)
print(myGraph.arestas)
print(myGraph.vertices)

print(myGraph.arestas[2])
print(myGraph.vertices[1])
