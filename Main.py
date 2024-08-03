# Trabalho prático da disciplina GCC218 - Algoritmos em Grafos
# Alunos: Guilherme Luiz de Azevedo, Henrique Assis Moreira, Mateus Piassi de Carvalho
# Universidade Federal de Lavras - UFLA, 2024/01

import re

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
    def __init__(self, is_directed):
        self.vertices = []
        self.edges = []
        directed = is_directed

    # Adds new vertices to the graph
    def add_vertice(self, vertice) -> None:
        if vertice not in self.vertices:
            self.vertices.append(vertice)

    # Adds new edges to the graph
    def add_edge(self, u, v) -> None:
        if (u, v) not in self.edges:
            self.edges.append((u, v))

    # Adds new attributte to the class
    def add_attribute(self, attribute_name, value) -> None:
        setattr(self, attribute_name, value)

    # Verifies the usability of the .txt file content
    def graph_pattern(self, string) -> bool:
        test = r"^\s*V\s*=\s*{[0-9]?[,0-9]*};\s*A\s*=\s*{[\(0-9,0-9\)]?[,\(0-9,0-9\)]*};$"  #Graph ReGex
        result = re.match(test, string)
        return bool(result)

    # Reads the file content and returns its content
    def read_txt(self) -> str:
        file_name = input("Please, insert the .txt file name here: ")
        file = open(file_name, "r")
        file_content = file.read()
        file.close()

        if self.graph_pattern(file_content):
            return file_content
        else:
            return None

    # Reads the string containing the graph and creates its vertices and edges
    def read_graph_string(self) -> None:
        try:
            graph_string = self.read_txt()

            vertice_content = graph_string.split('V = ')[1].split(';')[0].strip()
            vertice_content = vertice_content.strip("{}")
            self.vertices = list(map(int, vertice_content.split(",")))

            edge_content = graph_string.split('A = ')[1].split(';')[0].strip()
            edge_content = edge_content.strip("{}")
            edge_array = []

            # Splits edge_content by '),('
            for x in edge_content.split('),('):
                x = x.strip("()")
                u, v = list(map(int, x.split(',')))  # Stores each edge member in 'u' and 'v'
                edge_array.append([u, v])  # Stores 'u' and 'v' in the edge_array list

            self.edges = edge_array
        except:
            print("File pattern is incorrect")

    # Create a graph matrix
    def create_matrix(self) -> list:
        self.read_graph_string()
        matrix_size = [0] * len(self.vertices)  # Cria uma lista com o tanto de 0 que tiver no vertice
        graph_matrix = []  # Inicializa a matriz do grafo

        for i in range(len(self.vertices)):
            graph_matrix.append(
                matrix_size.copy())  # Cria a matriz com cópias independentes da lista, criando uma matriz quadrada de 0

        for i in self.edges:
            line = i[0] - 1
            column = i[1] - 1
            graph_matrix[line][column] = 1  # Substitui a posição indicada pelo número 1, simbolizando que tem uma aresta

        return graph_matrix


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
