#Imports the ReGex module
import re

class Graph:
    
    #Class constructor
    def __init__(self, is_directed):
        self.vertices = []
        self.edges = []
        directed = is_directed
    
    #Adds new vertices to the graph
    def add_vertice(self, vertice) -> None:
        if vertice not in self.vertices:
            self.vertices.append(vertice)

    #Adds new edges to the graph
    def add_edge(self, u, v) -> None:
        if (u,v) not in self.edges:
            self.edges.append((u,v))

    #Adds new attributte to the class
    def add_attribute(self, attribute_name, value) -> None:
        setattr(self, attribute_name, value)                    

    #Verifies the usability of the .txt file content
    def graph_pattern(self, string) -> bool: 
        test = r"^\s*V\s*=\s*{[0-9]?[,0-9]*};\s*A\s*=\s*{[\(0-9,0-9\)]?[,\(0-9,0-9\)]*};$" #Graph ReGex
        result = re.match(test, string)
        return bool(result)
    
    #Reads the file content and returns its content
    def read_txt(self) -> str:
        file_name = input("Please, insert the .txt file name here: ")
        file = open(file_name, "r")
        file_content = file.read()
        file.close()

        if(self.graph_pattern(file_content)):
            return file_content
        else:
            return None
    
    #Reads the string containing the graph and creates its vertices and edges
    def read_graph_string(self) -> None: 
        try:
            graph_string = self.read_txt()

            vertice_content = graph_string.split('V = ')[1].split(';')[0].strip()
            vertice_content = vertice_content.strip("{}")
            self.vertices = list(map(int, vertice_content.split(",")))
        
            edge_content = graph_string.split('A = ')[1].split(';')[0].strip()
            edge_content = edge_content.strip("{}")
            edge_array = []
            
            #Splits edge_content by '),('
            for x in edge_content.split('),('): 
                x = x.strip("()")
                u, v = list(map(int, x.split(',')))         #Stores each edge member in 'u' and 'v'
                edge_array.append([u,v])                    #Stores 'u' and 'v' in the edge_array list | Piassi eu troquei os () por [] :3

            self.edges = edge_array
        except:
            print("File pattern is incorrect")

    #Create a graph matrix
    def create_matrix(self) -> list:
        self.read_graph_string()
        matrix_size = [0] * len(self.vertices)       #Cria uma lista com o tanto de 0 que tiver no vertice
        graph_matrix = []                            #Inicializa a matriz do grafo

        for i in range(len(self.vertices)):
            graph_matrix.append(matrix_size.copy())  #Cria a matriz com cópias independentes da lista, criando uma matriz quadrada de 0

        for i in self.edges:
            line = i[0] - 1
            column = i[1] - 1
            graph_matrix[line][column] = 1           #Substitui a posição indicada pelo número 1, simbolizando que tem uma aresta

        return graph_matrix