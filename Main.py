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
    
    def tem_ciclo(self): #função que verifica se há um ciclo no grafo
        visitados = [False] * self.n_vertices #vetor com os ertices visitados em cada iteração, para garantir a verificação de todos os conjuntos

        for i in range(self.n_vertices):
            if not visitados[i]:
                if self.dfs_ciclo(i, visitados):
                    return True
        return False

    def dfs_ciclo(self, start, visitados): #dfs que verifica a existência de ciclos
        visitados_atual = [False] * self.n_vertices 
        pilha = [start]
        pais = [-1] * self.n_vertices  #vetor que registra o pai de cada vértice
        pais[start] = -1

        while pilha:
            v_atual = pilha.pop()
            visitados[v_atual] = True
            visitados_atual[v_atual] = True
            vetor_ancestrais = self.retorna_ancestrais(v_atual, pais) #retorna os ancestrais do vertice atual

            for vizinho in range(self.n_vertices):
                if self.matriz_adjacencia[v_atual][vizinho] != -1:
                    pais[vizinho] = v_atual
                    if vizinho in vetor_ancestrais:
                        return True # se o vizinho for um dos ancestrais, achamos um ciclo
                    if vizinho not in visitados_atual:
                        pilha.append(vizinho)

        return False
        
    def retorna_ancestrais(self, inicial, pais): #função que retorna os ancestrais de uma folha em relação a raiz
        vetor_ancestrais = []
        filho = inicial
        pai = 0
        while pai != -1:
            pai = pais[filho]
            vetor_ancestrais.append(pai)
            filho = pai

        return vetor_ancestrais 
    
    def conta_componentes_conexas(self): #retorna quantos componentes conexos tem um grafo não orientado
        visitados = [False] * self.n_vertices
        n_componentes = 0

        for v in range(self.n_vertices):
            if not visitados[v]:
                self.dfs_componentes(v, visitados) # faz uma dfs e conta quantas iterações foram necessárias para passar por todos os componentes
                n_componentes += 1

        return n_componentes

    def dfs_componentes(self, start, visitados):
        pilha = [start]

        while pilha:
            v = pilha.pop()
            if not visitados[v]:
                visitados[v] = True

                # Adiciona todos os vizinhos não visitados à pilha
                for vizinho in range(self.n_vertices):
                    if self.matriz_adjacencia[v][vizinho] != -1 and not visitados[vizinho]:
                        pilha.append(vizinho)
    
    def kosaraju(self): #verifica quantos componentes conexos tem um grafo orientado
        # Passo 1: Fazer a primeira DFS para calcular a ordem de término dos vértices
        visitados = [False] * self.n_vertices
        ordem_termino = []

        for v in range(self.n_vertices):
            if not visitados[v]:
                self.dfs_ordem_termino(v, visitados, ordem_termino)

        # Passo 2: Transpor o grafo
        grafo_transposto = self.transpor_grafo()

        # Passo 3: Fazer DFS no grafo transposto na ordem inversa da ordem de término
        visitados = [False] * self.n_vertices
        scc_count = 0

        while ordem_termino:
            v = ordem_termino.pop()
            if not visitados[v]:
                grafo_transposto.dfs_visit(v, visitados)
                scc_count += 1  # Cada DFS completa encontra uma componente fortemente conexa

        return scc_count

    def dfs_ordem_termino(self, start, visitados, ordem_termino):
        pilha = [start]
        visitados_locais = [False] * self.n_vertices  # Para controlar os nós já processados localmente

        while pilha:
            v = pilha[-1]  # Pega o vértice do topo da pilha
            if not visitados[v]:
                visitados[v] = True
                visitados_locais[v] = False  # Marca que ainda não foi processado localmente

            # Itera sobre os vizinhos
            for vizinho in range(self.n_vertices):
                if self.matriz_adjacencia[v][vizinho] != -1 and not visitados[vizinho]:
                    pilha.append(vizinho)
                    break
            else:
                # Se todos os vizinhos foram visitados ou não há mais vizinhos
                if not visitados_locais[v]:
                    ordem_termino.append(v)
                    visitados_locais[v] = True
                pilha.pop()

    def transpor_grafo(self):
        grafo_transposto = Graph()
        grafo_transposto.n_vertices = self.n_vertices
        grafo_transposto.is_direcionado = True

        grafo_transposto.matriz_adjacencia = [[-1 for _ in range(self.n_vertices)] for _ in range(self.n_vertices)]

        for v1 in range(self.n_vertices):
            for v2 in range(self.n_vertices):
                if self.matriz_adjacencia[v1][v2] != -1:
                    grafo_transposto.matriz_adjacencia[v2][v1] = self.matriz_adjacencia[v1][v2]

        return grafo_transposto

    def dfs_visit(self, start, visitados):
        pilha = [start]

        while pilha:
            v = pilha.pop()
            if not visitados[v]:
                visitados[v] = True

            # Adiciona os vizinhos não visitados na pilha
            for vizinho in range(self.n_vertices - 1, -1, -1):  # Ordem inversa para simular a chamada recursiva
                if self.matriz_adjacencia[v][vizinho] != -1 and not visitados[vizinho]:
                    pilha.append(vizinho)   

    def conta_pontes(self): #conta as arestas pontes existentes em um grafo
        # Inicializações
        self.tempo = 0  # Tempo de descoberta na DFS
        visitados = [False] * self.n_vertices
        discovery = [-1] * self.n_vertices  # Tempo de descoberta dos vértices
        low = [-1] * self.n_vertices  # Valor low dos vértices
        parent = [-1] * self.n_vertices  # Vetor para armazenar os pais dos vértices na DFS
        bridges = 0  # Contador de arestas ponte

        for i in range(self.n_vertices):
            if not visitados[i]:
                bridges += self.dfs_conta_pontes(i, visitados, discovery, low, parent)

        return bridges

    def dfs_conta_pontes(self, u, visitados, discovery, low, parent):
        visitados[u] = True
        discovery[u] = low[u] = self.tempo
        self.tempo += 1
        bridges = 0

        for v in range(self.n_vertices):
            if self.matriz_adjacencia[u][v] != -1:  # Verifica adjacência
                if not visitados[v]:  # Se o vizinho não foi visitado, explorar
                    parent[v] = u
                    bridges += self.dfs_conta_pontes(v, visitados, discovery, low, parent)

                    # Atualiza o valor de low para o vértice atual
                    low[u] = min(low[u], low[v])

                    # Verifica se a aresta (u, v) é uma ponte
                    if low[v] > discovery[u]:
                        bridges += 1

                elif v != parent[u]:  # Atualiza low[u] para arestas de retorno
                    low[u] = min(low[u], discovery[v])

        return bridges
    def imprime_arvore_profundidade(self): #DFS que imprime o id das arestas utilizadas na busca
        visitados = [False] * self.n_vertices
        pilha = [(0, -1)]  # pilha com tuplas (vértice atual, aresta usada para chegar aqui)
        arestas_arvore = []

        while pilha:
            v_atual, id_aresta = pilha.pop()
            if not visitados[v_atual]:
                visitados[v_atual] = True
                if id_aresta != -1:
                    arestas_arvore.append(id_aresta) #adiciona na lista o id da aretesa utilizada pata chegar em v_atual

                vizinhos = []
                for vizinho in range(self.n_vertices):
                    if self.matriz_adjacencia[v_atual][vizinho] != -1 and not visitados[vizinho]:
                        id_aresta_vizinho = self.get_edge_id(v_atual, vizinho)
                        vizinhos.append((vizinho, id_aresta_vizinho))

                vizinhos.sort()  # Ordena os vizinhos em ordem lexicográfica
                pilha.extend(vizinhos[::-1])  # Adiciona os vizinhos à pilha na ordem lexicográfica

        # Imprimir os identificadores das arestas da árvore
        string_id_aresta = ''
        for id_aresta in sorted(arestas_arvore):
            string_id_aresta += str(id_aresta) + '  '
        
        return string_id_aresta
    
    def imprime_arvore_largura(self): #BFS que imprime o id das arestas utilizadas na busca
        visitados = [False] * self.n_vertices
        fila = [(0, -1)]  # fila com tuplas (vértice atual, aresta usada para chegar aqui)
        arestas_arvore = []

        while fila:
            v_atual, id_aresta = fila.pop()
            if not visitados[v_atual]:
                visitados[v_atual] = True
                if id_aresta != -1:
                    arestas_arvore.append(id_aresta) #adiciona na lista o id da aretesa utilizada pata chegar em v_atual

                vizinhos = []
                for vizinho in range(self.n_vertices):
                    if self.matriz_adjacencia[v_atual][vizinho] != -1 and not visitados[vizinho]:
                        id_aresta_vizinho = self.get_edge_id(v_atual, vizinho)
                        vizinhos.append((vizinho, id_aresta_vizinho))

                vizinhos.sort()  # Ordena os vizinhos em ordem lexicográfica
                fila.extend(vizinhos[::-1])  # Adiciona os vizinhos à pilha na ordem lexicográfica

        # Imprimir os identificadores das arestas da árvore
        string_id_aresta = ''
        for id_aresta in sorted(arestas_arvore):
            string_id_aresta += str(id_aresta) + '  '
        
        return string_id_aresta


    def get_edge_id(self, v1, v2): #pega o id de aresta através do par de vertices
        if self.is_direcionado:
            for edge in self.edges:
                if(edge[1] == v1 and edge[2] == v2):
                    return edge[0]
        else:
            for edge in self.edges:
                if (edge[1] == v1 and edge[2] == v2) or (edge[1] == v2 and edge[2] == v1):
                    return edge[0]
                
        return -1


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