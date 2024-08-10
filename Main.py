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

    # Class constructor
    def __init__(self):
        self.vertices = []
        self.edges = [] #formato: [indice, v1, v2, peso]
        self.matriz_adjacencia = []
        self.n_vertices = 0
        self.n_edges = 0
        self.is_direcionado = True

    #função para entrada dos dados
    def read_graph_data(self) -> None:
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

    #cria uma matriz de adjacência pro grafo
    def define_matriz_adjacencia(self):
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
            for i in range(self.n_vertices):
                for j in range(self.n_vertices):
                    if(matrix[i][j] != -1):
                        matrix[j][i] = matrix[i][j]
        
        self.matriz_adjacencia = matrix

    #Realiza uma DFS que retorna a ordem dos vertices percorridos
    def DFS_conexo(self, v_inicial):
        visitados = []
        pilha = [v_inicial]

        while pilha:
            v_atual = pilha.pop()
            if v_atual not in visitados:
                visitados.append(v_atual)
                for neighbor in range(self.n_vertices):
                    if self.matriz_adjacencia[v_atual][neighbor] != -1 and neighbor not in visitados:
                        pilha.append(neighbor)

        return visitados
    
    #verifica se o tamanho do vetor de vértices retornado pela DFS é igual ao número de vértices do grafo
    def verifica_conexidade (self):
         n_visitados = len(self.DFS_conexo(self.vertices[0]))
         if n_visitados != self.n_vertices:
             return False
         else:
             return True

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