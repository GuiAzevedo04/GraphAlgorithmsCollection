#Importa o modulo de ReGex
import re

class Grafo:
    def __init__(self):
        pass
    
    #Função que adiciona um atributo na classe
    def adicionar_atributo(self, nome_atributo, valor) -> None: 
        setattr(self, nome_atributo, valor)

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
        
        #Cria um novo atributo chamado 'vertices'
        self.adicionar_atributo('vertices', vert_content)                 

        edge_content = graph_string.split('A = ')[1].split(';')[0].strip()
        edge_content = edge_content.strip("{}")
        edge_array = []

        #Divide edge_content por '),('
        for x in edge_content.split('),('): 
            x = x.strip("()")
            u, v = x.split(',')         #Guarda cada membro da aresta em u e v
            edge_array.append((u,v))    #Guarda u e v na lista

        #Cria um novo atributo chamado 'arestas'
        self.adicionar_atributo('arestas', edge_array) 
