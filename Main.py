# Trabalho prático da disciplina GCC218 - Algoritmos em Grafos
# Alunos: Guilherme Luiz de Azevedo, Henrique Assis Moreira, Mateus Piassi de Carvalho
# Universidade Federal de Lavras - UFLA, 2024/01


class Graph:

    def __init__(self):  # Class constructor
        self.vertices = []
        self.edges = []  # formato: [indice, v1, v2, peso]
        self.matriz_adjacencia = []
        self.n_vertices = 0
        self.n_edges = 0
        self.is_direcionado = True
        self.instrucoes = []

    def read_graph_data(self) -> None:  # função para entrada dos dados
        try:

            self.instrucoes = list(map(int, input("").split()))

            self.n_vertices, self.n_edges = map(int, input("Enter number of vertices and edges: ").split())
            direcionado_str = input("Is the graph directed (digite 'nao_direcionado' para não direcionado)? ").strip()

            if direcionado_str == 'nao_direcionado':
                self.is_direcionado = False

            print("Enter each edge in the format: [index] [vertex1] [vertex2] [weight]")
            for x in range(self.n_edges):
                self.edges.append(list(input().split(' ')))
                for i in range(len(self.edges[x])):
                    self.edges[x][i] = int(self.edges[x][i])

            for x in self.edges:
                temp_vert1 = x[1]
                temp_vert2 = x[2]

                if temp_vert1 not in self.vertices:
                    self.vertices.append(temp_vert1)

                if temp_vert2 not in self.vertices:
                    self.vertices.append(temp_vert2)

            self.vertices.sort()

        except ValueError as e:
            print(f"Houve um erro na leitura: {e}")

    def define_matriz_adjacencia(self):  # cria uma matriz de adjacência pro grafo
        matrix = []

        for _ in self.vertices:
            insert = []
            for _ in self.vertices:
                insert.append(-1)

            matrix.append(insert)

        for x in self.edges:
            vetor_temp = x
            vertice1 = int(vetor_temp[1])
            vertice2 = int(vetor_temp[2])

            matrix[vertice1][vertice2] = int(vetor_temp[3])

        if not self.is_direcionado:  # torna a matriz simétrica
            matrix = self.faz_matriz_simetrica(matrix)

        self.matriz_adjacencia = matrix

    def faz_matriz_simetrica(self, matrix):  # retorna uma matriz simétrica
        for i in range(self.n_vertices):
            for j in range(self.n_vertices):
                if matrix[i][j] != -1:
                    matrix[j][i] = matrix[i][j]

        return matrix

    def dfs_vertices_percorridos(self, v_inicial, matrix):  # DFS que retorna os vértices percorridos
        visitados = []
        pilha = [v_inicial]

        while pilha:
            v_atual = pilha.pop()
            if v_atual not in visitados:
                visitados.append(v_atual)
                for neighbor in range(self.n_vertices):
                    if matrix[v_atual][neighbor] != -1 and neighbor not in visitados:
                        pilha.append(neighbor)

        return visitados

    def verifica_conexidade(self):
        if not self.is_direcionado:  # se o grafo não for direcionado, verifica se normalmente
            matrix_temp = self.matriz_adjacencia
        else:  # se for direcionado, verifica-se sua conexidade fraca ignorando sua conexidade
            matrix_temp = self.faz_matriz_simetrica(self.matriz_adjacencia)
        # se o tamano do vetor retornado pela DFS não for igual ao número de vértices o grafo não é conexo
        n_visitados = len(self.dfs_vertices_percorridos(self.vertices[0], matrix_temp))
        if n_visitados != self.n_vertices:
            return False
        else:
            return True

    def verifica_bipartido(self):  # verifica se o grafo é bipartido, usando o metodo de coloração
        color = [-1] * self.n_vertices  # Inicializar um vetor de cores onde -1 indica que o vértice não foi colorido

        for i in range(self.n_vertices):
            if color[i] == -1:  # Se o vértice não foi colorido, iniciar BFS
                if not self.verifica_componente_bipartido(i, color):
                    return False

        return True

    # a partir de um vertice inicial, verifica se o componente é bipartido
    def verifica_componente_bipartido(self, start, color):
        if not self.is_direcionado:
            # usar BFS para colorir os vértices
            queue = [start]
            color[start] = 0  # iniciar com a primeira cor

            while queue:
                v = queue.pop(0)

                for neighbor in range(self.n_vertices):
                    if self.matriz_adjacencia[v][neighbor] != -1:  # Verificar adjacência
                        if color[neighbor] == -1:
                            # colorir o vizinho com a cor oposta
                            color[neighbor] = 1 - color[v]
                            queue.append(neighbor)
                        elif color[neighbor] == color[v]:
                            # se um vizinho tem a mesma cor, o grafo não é bipartido
                            return False

            return True
        else:
            return False

    def verifica_euleriano(self):
        if not self.verifica_conexidade():
            return False  # o grafo não é conexo, então não é euleriano

        if self.is_direcionado:
            # para grafos direcionados, verificar se todos os vértices têm grau de entrada igual ao grau de saída
            grau_entrada = [0] * self.n_vertices
            grau_saida = [0] * self.n_vertices

            for edge in self.edges:
                v1, v2 = edge[1], edge[2]
                grau_saida[v1] += 1
                grau_entrada[v2] += 1

            for i in range(self.n_vertices):
                if grau_entrada[i] != grau_saida[i]:
                    return False

        else:
            # para grafos não direcionados, verificar se todos os vértices têm grau par
            grau = [0] * self.n_vertices

            for edge in self.edges:
                v1, v2 = edge[1], edge[2]
                grau[v1] += 1
                grau[v2] += 1

            for g in grau:
                if g % 2 != 0:
                    return False

        return True  # se todas as condições forem atendidas, o grafo é euleriano


def main():

    close_program = False
    meu_grafo = Graph()
    meu_grafo.read_graph_data()

    while not close_program:

        match meu_grafo.instrucoes[0]:
            case 0:
                print("ToDo instrucao 0")
            case 1:
                print("ToDo instrucao 1")
            case 2:
                print("ToDo instrucao 2")
            case 3:
                print("ToDo instrucao 3")
            case 4:
                print("ToDo instrucao 4")
            case 5:
                print("ToDo instrucao 5")
            case 6:
                print("ToDo instrucao 6")
            case 7:
                print("ToDo instrucao 7")
            case 8:
                print("ToDo instrucao 8")
            case 9:
                print("ToDo instrucao 9")
            case 10:
                print("ToDo instrucao 10")
            case 11:
                print("ToDo instrucao 11")
            case 12:
                print("ToDo instrucao 12")
            case 13:
                print("ToDo instrucao 13")
            case 14:
                print("ToDo instrucao 14")
            case _:
                print("Invalid Command")

        meu_grafo.instrucoes.pop(0)

        if len(meu_grafo.instrucoes) == 0:
            close_program = True


if __name__ == "__main__":
    main()