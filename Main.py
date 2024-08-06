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
        self.edges = []
        self.is_direcionado = True

    # Reads the file content and returns its content
    def read_txt(self) -> str:
        file_name = input("Please, insert the .txt file name here: ")
        try:
            file = open(file_name + '.txt', "r")                
            file_content = file.read()
            file.close()

            return file_content
        except:
            return None

    # Reads the string containing the graph and creates its vertices and edges
    def read_graph_string(self) -> None:
        graph_string = self.read_txt()

        try:
            infos = graph_string.split('\n')
            n_vertices = infos[0].split(' ')[0]
            n_arestas = int(infos[0].split(' ')[1])

            if(infos[1] == 'nao_direcionado'):
                self.is_direcionado = False
            
            for x in range(2, n_arestas + 2):
                self.edges.append(infos[x])

            for x in range(n_vertices):
                self.vertices.append(x)

        except:
            print('Houve um erro na leitura')


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