# Trabalho prático da disciplina GCC218 - Algoritmos em Grafos
# Alunos: Guilherme Luiz de Azevedo, Henrique Assis Moreira, Mateus Piassi de Carvalho
# Universidade Federal de Lavras - UFLA, 2024/01

def interface_text() -> None:
    print(f"[0] Read new .txt file: ")
    print(f"[1] Make Verifications")
    print(f"[2] List components")
    print(f"[3] Generate Adjacency matrix")
    print(f"[4] Generate Adjacency list")
    print(f"[5] Generate Depth Tree")
    print(f"[6] Generate Minimum spanning tree")
    print(f"[7] Topological sort (This function is not available in undirected graphs)")
    print(f"[8] Shortest path between two vertices (This function is not available in unweighted graphs)")
    print(f"[9] Transitive closure (This function is not available in unweighted graphs): ")
    print(f"[C] Close program.")

class Graph:

    def __init__(self): # Class constructor
        self.vertices = []
        self.edges = [] #formato: [indice, v1, v2, peso]
        self.matriz_adjacencia = []
        self.n_vertices = 0
        self.n_edges = 0
        self.is_direcionado = True

    def read_graph_data(self) -> None: #função para entrada dos dados
        try:
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

    def define_matriz_adjacencia(self):#cria uma matriz de adjacência pro grafo
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

        if self.is_direcionado == False: #torna a matriz simétrica
            matrix = self.faz_matriz_simetrica(matrix)
        
        self.matriz_adjacencia = matrix

    def faz_matriz_simetrica(self, matrix): #retorna uma matriz simétrica
        for i in range(self.n_vertices):
                for j in range(self.n_vertices):
                    if(matrix[i][j] != -1):
                        matrix[j][i] = matrix[i][j]

        return matrix

    def DFS_vertices_percorridos(self, v_inicial, matrix): #DFS que retorna os vértices percorridos
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
    
    def verifica_conexidade (self):
        if (self.is_direcionado == False): #se o grafo não for direcionado, verifica se normalmente
            matrix_temp = self.matriz_adjacencia
        else: #se for direcionado, verifica-se sua conexidade fraca ignorando sua conexidade
            matrix_temp = self.faz_matriz_simetrica(self.matriz_adjacencia)
        #se o tamano do vetor retornado pela DFS não for igual ao número de vértices o grafo não é conexo
        n_visitados = len(self.DFS_vertices_percorridos(self.vertices[0], matrix_temp)) 
        if n_visitados != self.n_vertices:
            return False
        else:
            return True    
        
    def verifica_bipartido(self): #verifica se o grafo é bipartido, usando o metodo de coloração
        color = [-1] * self.n_vertices # Inicializar um vetor de cores onde -1 indica que o vértice não foi colorido
        
        for i in range(self.n_vertices):
            if color[i] == -1:  # Se o vértice não foi colorido, iniciar BFS
                if not self.is_bipartite_component(i, color):
                    return False
        
        return True

    def is_bipartite_component(self, start, color): #a partir de um vertice inicial, verifica se o componente é bipartido
        if(self.is_direcionado == False):
            #usar BFS para colorir os vértices
            queue = [start]
            color[start] = 0  #iniciar com a primeira cor

            while queue:
                v = queue.pop(0)

                for neighbor in range(self.n_vertices):
                    if self.matriz_adjacencia[v][neighbor] != -1:#Verificar adjacência
                        if color[neighbor] == -1:
                            #colorir o vizinho com a cor oposta
                            color[neighbor] = 1 - color[v]
                            queue.append(neighbor)
                        elif color[neighbor] == color[v]:
                            #se um vizinho tem a mesma cor, o grafo não é bipartido
                            return False
        
            return True
        else: return False

    
def main():
    user_input = str
    close_program = False
    while not close_program:

        interface_text()
        user_input = input("").lower()

        match user_input:
            case '0':
                print("ToDo")
            case '1':
                print("ToDo")
            case '2':
                print("ToDo")
            case '3':
                print("ToDo")
            case '4':
                print("ToDo")
            case '5':
                print("ToDo")
            case '6':
                print("ToDo")
            case '7':
                print("ToDo")
            case '8':
                print("ToDo")
            case '9':
                print("ToDo")
            case 'c':
                close_program = True
            case _:
                print("Invalid Command")