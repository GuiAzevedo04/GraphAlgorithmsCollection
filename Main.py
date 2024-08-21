# Trabalho prático da disciplina GCC218 - Algoritmos em Grafos
# Alunos: Guilherme Luiz de Azevedo, Henrique Assis Moreira, Mateus Piassi de Carvalho
# Universidade Federal de Lavras - UFLA, 2024/01
import copy
import heapq


class Graph:

    def __init__(self):  # Class constructor
        self.vertices = []
        self.edges = []  # formato: [indice, v1, v2, peso]
        self.matriz_adjacencia = []
        self.matriz_simetrica = []
        self.n_vertices = 0
        self.n_edges = 0
        self.is_direcionado = True
        self.instrucoes = []
        self.lista_adjacencia = []

    def read_graph_data(self) -> None:  # procedimento para entrada dos dados
        try:

            # Lê as informações do Grafo
            self.instrucoes = list(map(int, input().split()))

            self.n_vertices, self.n_edges = map(int, input().split())
            direcionado_str = input().strip()

            if direcionado_str == 'nao_direcionado':
                self.is_direcionado = False

            for x in range(self.n_edges):
                self.edges.append(list(input().split(' ')))
                for i in range(len(self.edges[x])):
                    self.edges[x][i] = int(self.edges[x][i])

            for x in range(self.n_vertices):
                self.vertices.append(x)

            # Cria as matrizes de adjacência e simétrica
            self.matriz_adjacencia = self.cria_matriz_adjacencia(self.n_vertices, self.edges)
            self.matriz_simetrica = self.cria_matriz_simetrica()
            self.lista_adjacencia = self.cria_lista_adjacencia()

        except ValueError as e:
            print(f"Houve um erro na leitura: {e}")

    def cria_lista_adjacencia(self) -> list:
        lista_adjacencia = [[] for _ in range(self.n_vertices)]

        for edge in self.edges:
            _, u, v, _ = edge
            lista_adjacencia[u].append(v)
            lista_adjacencia[v].append(u)

        return lista_adjacencia

    def cria_matriz_adjacencia(self, n_vertices, arestas) -> list:
        # Inicializa a matriz de adjacência com -1
        matrix = [[-1 for _ in range(n_vertices)] for _ in range(n_vertices)]

        # Popula a matriz com os pesos das arestas
        for aresta in arestas:
            _, u, v, peso = aresta
            matrix[u][v] = peso

        return matrix

    def cria_matriz_simetrica(self) -> list:
        matriz_simetrica = copy.deepcopy(self.matriz_adjacencia)
        for i in range(self.n_vertices):
            for j in range(i + 1, self.n_vertices):  # Percorre apenas a metade superior da matriz
                if matriz_simetrica[i][j] != -1 and matriz_simetrica[j][i] == -1:
                    matriz_simetrica[j][i] = matriz_simetrica[i][j]
                elif matriz_simetrica[j][i] != -1 and matriz_simetrica[i][j] == -1:
                    matriz_simetrica[i][j] = matriz_simetrica[j][i]

        return matriz_simetrica

    def dfs_vertices_percorridos(self, v_inicial, matrix) -> list[int]:  # DFS que retorna os vértices percorridos
        visitados = set()
        pilha = [v_inicial]

        while pilha:
            v_atual = pilha.pop()
            if v_atual not in visitados:
                visitados.add(v_atual)
                for neighbor in range(self.n_vertices):
                    if matrix[v_atual][neighbor] != -1 and neighbor not in visitados:
                        pilha.append(neighbor)

        return list(visitados)

    def verifica_conexidade(self) -> bool:
        if not self.is_direcionado:  # se o grafo não for direcionado, verifica se normalmente
            matrix_temp = self.matriz_adjacencia
        else:  # se for direcionado, verifica-se sua conexidade fraca ignorando sua conexidade
            matrix_temp = self.matriz_simetrica
        # se o tamano do vetor retornado pela DFS não for igual ao número de vértices o grafo não é conexo
        n_visitados = len(self.dfs_vertices_percorridos(self.vertices[0], matrix_temp))
        return n_visitados == self.n_vertices

    def verifica_bipartido(self) -> bool:  # verifica se o grafo é bipartido, usando o metodo de coloração
        color = [-1] * self.n_vertices  # Inicializar um vetor de cores onde -1 indica que o vértice não foi colorido

        for i in range(self.n_vertices):
            if color[i] == -1:  # Se o vértice não foi colorido, iniciar BFS
                if not self.verifica_componente_bipartido(i, color):
                    return False

        return True

    # a partir de um vertice inicial, verifica se o componente é bipartido
    def verifica_componente_bipartido(self, start, color) -> bool:
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

    def verifica_euleriano(self) -> bool:
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

    def tem_ciclo(self) -> bool:  # função que verifica se há um ciclo no grafo
        visitados = [False] * self.n_vertices  # vetor com os ertices visitados em cada iteração, para garantir a verificação de todos os conjuntos

        for i in range(self.n_vertices):
            if not visitados[i]:
                if self.dfs_ciclo(i, visitados):
                    return True
        return False

    def dfs_ciclo(self, start, visitados) -> bool:  # dfs que verifica a existência de ciclos
        visitados_atual = [False] * self.n_vertices
        pilha = [start]
        pais = [-1] * self.n_vertices  # vetor que registra o pai de cada vértice
        pais[start] = -1

        while pilha:
            v_atual = pilha.pop()
            visitados[v_atual] = True
            visitados_atual[v_atual] = True
            vetor_ancestrais = self.retorna_ancestrais(v_atual, pais)  # retorna os ancestrais do vertice atual

            for vizinho in range(self.n_vertices):
                if self.matriz_adjacencia[v_atual][vizinho] != -1:
                    pais[vizinho] = v_atual
                    if vizinho in vetor_ancestrais:
                        return True  # se o vizinho for um dos ancestrais, achamos um ciclo
                    if vizinho not in visitados_atual:
                        pilha.append(vizinho)

        return False

    def retorna_ancestrais(self, inicial, pais) -> list:  # retorna os ancestrais de uma folha em relação a raiz
        vetor_ancestrais = []
        filho = inicial
        pai = 0
        while pai != -1:
            pai = pais[filho]
            vetor_ancestrais.append(pai)
            filho = pai

        return vetor_ancestrais

    def conta_componentes_conexas(self) -> int:  # retorna quantos componentes conexos tem um grafo não orientado
        visitados = [False] * self.n_vertices
        n_componentes = 0

        for v in range(self.n_vertices):
            if not visitados[v]:
                self.dfs_componentes(v, visitados)  # faz uma dfs e conta quantas iterações foram necessárias para passar por todos os componentes
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

    def kosaraju(self) -> int:  # verifica quantos componentes conexos tem um grafo orientado
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

    def lista_articulacoes(self, lista_adjacencia, n_vertices) -> list[str]:
        baixos = [-1] * n_vertices
        visitados = [False] * n_vertices
        pontos_de_articulacao = set()
        pais = [None] * n_vertices
        tempo_descoberta = [-1] * n_vertices
        tempo = 0

        def dfs_articulacoes(u, pais, visitados, tempo_descoberta, baixos, pontos_de_articulacao, tempo, lista_adjacencia):

            tempo += 1
            tempo_descoberta[u] = baixos[u] = tempo
            visitados[u] = True
            filhos = 0

            for v in lista_adjacencia[u]:
                if not visitados[v]:
                    filhos += 1
                    pais[v] = u
                    dfs_articulacoes(v, pais, visitados, tempo_descoberta, baixos, pontos_de_articulacao, tempo, lista_adjacencia)
                    baixos[u] = min(baixos[u], baixos[v])

                    if pais[u] is None and filhos > 1:
                        pontos_de_articulacao.add(u)
                    if pais[u] is not None and baixos[v] >= tempo_descoberta[u]:
                        pontos_de_articulacao.add(u)
                elif v != pais[u]:
                    baixos[u] = min(baixos[u], tempo_descoberta[v])

        for i in range(n_vertices):
            if not visitados[i]:
                dfs_articulacoes(i, pais, visitados, tempo_descoberta, baixos, pontos_de_articulacao, tempo, lista_adjacencia)

        lista_de_retorno = list(pontos_de_articulacao)
        lista_de_retorno.append(0)
        lista_de_retorno = ' '.join(map(str, sorted(lista_de_retorno)))

        return lista_de_retorno

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

    def transpor_grafo(self):  # Transpõe o grafo
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

    def conta_pontes(self) -> int:  # conta as arestas pontes existentes em um grafo
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

    def dfs_conta_pontes(self, u, visitados, discovery, low, parent) -> int:
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

    def imprime_arvore_profundidade(self) -> str:  # DFS que imprime o id das arestas utilizadas na busca
        visitados = [False] * self.n_vertices
        pilha = [(0, -1)]  # pilha com tuplas (vértice atual, aresta usada para chegar aqui)
        arestas_arvore = []

        while pilha:
            v_atual, id_aresta = pilha.pop()
            if not visitados[v_atual]:
                visitados[v_atual] = True
                if id_aresta != -1:
                    arestas_arvore.append(id_aresta)  # adiciona na lista o id da aretesa utilizada pata chegar em v_atual

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

    def imprime_arvore_largura(self) -> str:  # BFS que imprime o id das arestas utilizadas na busca
        visitados = [False] * self.n_vertices
        fila = [(0, -1)]  # fila com tuplas (vértice atual, aresta usada para chegar aqui)
        arestas_arvore = []

        while fila:
            v_atual, id_aresta = fila.pop(0)
            if not visitados[v_atual]:
                visitados[v_atual] = True
                if id_aresta != -1:
                    arestas_arvore.append(id_aresta)  # adiciona na lista o id da aretesa utilizada pata chegar em v_atual

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

    def get_edge_id(self, v1, v2):  # pega o id de aresta através do par de vertices
        if self.is_direcionado:
            for edge in self.edges:
                if edge[1] == v1 and edge[2] == v2:
                    return edge[0]
        else:
            for edge in self.edges:
                if (edge[1] == v1 and edge[2] == v2) or (edge[1] == v2 and edge[2] == v1):
                    return edge[0]

        return -1

    def prim(self) -> int:
        if not self.edges:
            return -1

        # Verifica se todas as arestas têm o mesmo peso
        try:
            weights = {peso for _, _, _, peso in self.edges}
        except ValueError:
            return -1

        if len(weights) == 1:
            return -1

        # Lista de adjacência
        adj = {i: [] for i in range(self.n_vertices)}
        for edge in self.edges:
            if len(edge) != 4:
                return -1
            id_aresta, u, v, peso = edge
            if not (0 <= u < self.n_vertices and 0 <= v < self.n_vertices):
                return -1
            adj[u].append((peso, v, id_aresta))
            adj[v].append((peso, u, id_aresta))

        # Inicializa a árvore geradora mínima
        mst_weight = 0
        visitado = [False] * self.n_vertices
        min_heap = [(0, 0, -1)]  # (peso, vértice, id_aresta)
        mst_edges = []

        while min_heap and len(mst_edges) < self.n_vertices - 1:
            peso, u, id_aresta = heapq.heappop(min_heap)

            if visitado[u]:
                continue

            visitado[u] = True
            if id_aresta != -1:
                mst_weight += peso
                mst_edges.append(id_aresta)

            for next_peso, next_v, next_id_aresta in adj[u]:
                if not visitado[next_v]:
                    heapq.heappush(min_heap, (next_peso, next_v, next_id_aresta))

        if len(mst_edges) == self.n_vertices - 1:
            return mst_weight
        else:
            return -1

    def has_equal_weight(self):
        if not self.edges:
            return True

        first_weight = self.edges[0][3]

        for _, _, _, weight in self.edges:
            if weight != first_weight:
                return False  # Se encontrar uma aresta com peso diferente, retorna False

        return True  # Se todas as arestas têm o mesmo peso, retorna True

    def dijkstra(self, start_vertex):
        # Inicializa as distâncias com infinito
        distances = {v: float('infinity') for v in range(self.n_vertices)}
        distances[start_vertex] = 0
        priority_queue = [(0, start_vertex)]

        while priority_queue:
            current_distance, current_vertex = heapq.heappop(priority_queue)

            # Verifica se a distância atual é maior do que a já encontrada
            if current_distance > distances[current_vertex]:
                continue

            # Explora as arestas conectadas ao vértice atual
            for edge_id, u, v, weight in self.edges:
                if u == current_vertex:
                    neighbor = v
                elif v == current_vertex:
                    neighbor = u
                else:
                    continue

                distance = current_distance + weight

                # Se a nova distância for menor, atualiza e adiciona na fila
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))

        return distances

    def fecho_transitivo_vertice(self, matriz_adjacencia):
        vertices = len(matriz_adjacencia)
        visitados = set()  # Usar um conjunto para garantir que não há duplicatas

        def dfs(v):
            for i in range(vertices):
                if matriz_adjacencia[v][i] == 1 and i not in visitados:
                    visitados.add(i)
                    dfs(i)

        # Executa DFS a partir do vértice 0
        dfs(0)

        return sorted(list(visitados))

    def shortest_path(self):
        if self.has_equal_weight():
            return -1

        start_vertex, end_vertex = 0, self.n_vertices - 1
        distances = self.dijkstra(start_vertex)
        return distances[end_vertex] if distances[end_vertex] != float('infinity') else -1

    def ordenacao_topologica(self, matriz_adjacencia) -> list:
        vertices = len(matriz_adjacencia)
        visitado = [False] * vertices
        pilha = []

        def dfs(v):
            visitado[v] = True
            for i in range(vertices):
                if matriz_adjacencia[v][i] == 1 and not visitado[i]:
                    dfs(i)
            pilha.append(v)

        # Executa DFS em todos os vértices para garantir que todos sejam visitados
        for v in range(vertices):
            if not visitado[v]:
                dfs(v)

        # Retorna a pilha invertida (a ordem topológica)

        pilha = ' '.join(map(str, pilha))

        return pilha

    def ford_fulkerson(self) -> int:
        # Inicializa o fluxo máximo
        max_flow = 0

        # Cria a matriz de capacidade
        capacity = [[0] * self.n_vertices for _ in range(self.n_vertices)]
        for edge in self.edges:
            u, v, capacity_value = edge[1], edge[2], edge[3]
            capacity[u][v] = capacity_value

        # Função auxiliar para encontrar um caminho aumentante usando BFS
        def bfs(source, sink, parent):
            visited = [False] * self.n_vertices
            queue = [source]
            visited[source] = True
            while queue:
                u = queue.pop(0)
                for v in range(self.n_vertices):
                    if not visited[v] and capacity[u][v] > 0:
                        parent[v] = u
                        visited[v] = True
                        if v == sink:
                            return True
                        queue.append(v)
            return False

        # Função para encontrar o fluxo mínimo no caminho encontrado
        def find_min_flow(source, sink, parent):
            path_flow = float('Inf')
            s = sink
            while s != source:
                path_flow = min(path_flow, capacity[parent[s]][s])
                s = parent[s]
            return path_flow

        # Função principal do algoritmo de Ford-Fulkerson
        source, sink = 0, self.n_vertices - 1
        parent = [-1] * self.n_vertices

        while bfs(source, sink, parent):
            path_flow = find_min_flow(source, sink, parent)
            max_flow += path_flow
            v = sink
            while v != source:
                u = parent[v]
                capacity[u][v] -= path_flow
                capacity[v][u] += path_flow
                v = parent[v]

        return max_flow


def main():

    close_program = False
    meu_grafo = Graph()
    meu_grafo.read_graph_data()

    while not close_program:

        match meu_grafo.instrucoes[0]:
            case 0:
                print(int(meu_grafo.verifica_conexidade()))
            case 1:
                print(int(meu_grafo.verifica_bipartido()))
            case 2:
                print(int(meu_grafo.verifica_euleriano()))
            case 3:
                print(int(meu_grafo.tem_ciclo()))
            case 4:
                if not meu_grafo.is_direcionado:
                    print(meu_grafo.conta_componentes_conexas())
                else:
                    print("-1")
            case 5:
                if meu_grafo.is_direcionado:
                    print(meu_grafo.kosaraju())
                else:
                    print("-1")
            case 6:
                if not meu_grafo.is_direcionado:
                    print(meu_grafo.lista_articulacoes(meu_grafo.lista_adjacencia, meu_grafo.n_vertices))
                else:
                    print("-1")
            case 7:
                if not meu_grafo.is_direcionado:
                    print(meu_grafo.conta_pontes())
                else:
                    print("-1")
            case 8:
                print(meu_grafo.imprime_arvore_profundidade())
            case 9:
                print(meu_grafo.imprime_arvore_largura())
            case 10:
                if not meu_grafo.is_direcionado:
                    print(meu_grafo.prim())
                else:
                    print("-1")
            case 11:
                print(meu_grafo.ordenacao_topologica(meu_grafo.matriz_adjacencia))
            case 12:
                if not meu_grafo.is_direcionado:
                    print(meu_grafo.shortest_path())
                else:
                    print("-1")
            case 13:
                print(meu_grafo.ford_fulkerson())
            case 14:
                if meu_grafo.is_direcionado:
                    print(meu_grafo.fecho_transitivo_vertice(meu_grafo.matriz_adjacencia))
                else:
                    print("-1")
            case _:
                print("Invalid Command")

        meu_grafo.instrucoes.pop(0)

        if len(meu_grafo.instrucoes) == 0:
            close_program = True


if __name__ == "__main__":
    main()