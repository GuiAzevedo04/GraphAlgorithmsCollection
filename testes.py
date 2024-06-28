from classGrafo import Grafo

meuGrafo = Grafo()
stringDoGrafo = meuGrafo.readtxt()

meuGrafo.readGraphString(stringDoGrafo)
print(meuGrafo.arestas)
print(meuGrafo.vertices)

print(meuGrafo.arestas[2])
print(meuGrafo.vertices[1])
