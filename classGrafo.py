#Importa o modulo de ReGex
import re

class Grafo:
    def __init__(self):
        self.vertices = []
        self.arestas = []
    
<<<<<<< HEAD
    def add_vertice(self, vert):
        if vert not in self.vertices:
            self.vertices.append(vert)

    def add_aresta(self, u, v):
        if (u,v) not in self.aresta:
            self.vertices.append((u,v))

    def adicionar_atributo(self, nome_atributo, valor): #função mágica que adiciona um atributo na classe
        setattr(self, nome_atributo, valor)             #é só adicionar o nome do atributo e seu valor
=======
    #Função que adiciona um atributo na classe
    def adicionar_atributo(self, nome_atributo, valor) -> None: 
        setattr(self, nome_atributo, valor)
>>>>>>> e85f36406c16ab26b59ce03ac2c51d92f237d365

    #Identifica se o padrão do .txt é válido
    def graph_pattern(self, string) -> bool: 
        test = r"^\s*V\s*=\s*{[0-9]?[,0-9]*};\s*A\s*=\s*{[\(0-9,0-9\)]?[,\(0-9,0-9\)]*};$" #ReGex do grafo
        result = re.match(test, string)
        return bool(result)
    
    #Lê o conteúdo do .txt e retorna seu conteúdo
    def read_txt(self) -> str: 
        name_arq = input("insira o nome do arquivo que contém o grafo: ")
        graph_arq = open(name_arq, "r")
        content = graph_arq.read()

        if(self.graph_pattern(content)):
            return content
        else:
            print("O padrão do arquivo está incorreto!")    
    
    #Lê a string e produz os vértices e arestas
    def read_graph_string(self, graph_string) -> None: 
        vert_content = graph_string.split('V = ')[1].split(';')[0].strip() #Separa em 2 partes usando "V = ", e pega a segunda parte.
        vert_content = vert_content.strip("{}")                            #Separa em 2 partes de novo, desta vez com ";", e pega a primeira parte
        vert_content = vert_content.split(",")                             #Por fim, elimina os espaços em branco em volta da string
        
<<<<<<< HEAD
    def readGraphString(self, graphString): #lê a string e produz os vértices e arestas
        vertContent = graphString.split('V = ')[1].split(';')[0].strip() #separa em 2 partes usando "V = ", e pega a segunda parte.
        vertContent = vertContent.strip("{}")                            #separa em 2 partes de novo, desta vez com ";", e pega a primeira parte
        self.vertices = list(map(int, vertContent.split(",")))           #por fim, elimina os espaços em branco em volta da string e guarda
        
=======
        #Cria um novo atributo chamado 'vertices'
        self.adicionar_atributo('vertices', vert_content)                 
>>>>>>> e85f36406c16ab26b59ce03ac2c51d92f237d365

        edge_content = graph_string.split('A = ')[1].split(';')[0].strip()
        edge_content = edge_content.strip("{}")
        edge_array = []

        #Divide edge_content por '),('
        for x in edge_content.split('),('): 
            x = x.strip("()")
<<<<<<< HEAD
            u, v = list(map(int, x.split(',')))            #guarda cada membro da aresta em u e v
            edgeArray.append((u,v))        #guarda u e v na lista

        self.arestas = edgeArray #guarda o array em arestas
=======
            u, v = x.split(',')         #Guarda cada membro da aresta em u e v
            edge_array.append((u,v))    #Guarda u e v na lista
>>>>>>> e85f36406c16ab26b59ce03ac2c51d92f237d365

        #Cria um novo atributo chamado 'arestas'
        self.adicionar_atributo('arestas', edge_array) 
