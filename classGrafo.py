#imports ReGex modules
import re

class Grafo:
    def __init__(self):
        pass
    
    #função que adiciona um atributo na classe
    def adicionar_atributo(self, nome_atributo, valor) -> None: 
        setattr(self, nome_atributo, valor)     #é só adicionar o nome do atributo e seu valor

    #identifica se o padrão do txt é válido
    def graphPattern(self, string) -> bool: 
        test = r"^\s*V\s*=\s*{[0-9]?[,0-9]*};\s*A\s*=\s*{[\(0-9,0-9\)]?[,\(0-9,0-9\)]*};$" #regex do grafo
        result = re.match(test, string)
        return bool(result)
    
    #lê o conteúdo do .txt e retorna seu conteúdo
    def readtxt(self) -> str: 
        nameArq = input("insira o nome do arquivo que contém o grafo: ")
        graphArq = open(nameArq, "r")
        content = graphArq.read()

        if(self.graphPattern(content)):
            return content
        else:
            print("O padrão do arquivo está incorreto!")    
    
    #lê a string e produz os vértices e arestas
    def readGraphString(self, graphString) -> None: 
        vertContent = graphString.split('V = ')[1].split(';')[0].strip() #separa em 2 partes usando "V = ", e pega a segunda parte.
        vertContent = vertContent.strip("{}")                            #separa em 2 partes de novo, desta vez com ";", e pega a primeira parte
        vertContent = vertContent.split(",")                             #por fim, elimina os espaços em branco em volta da string
        
        self.adicionar_atributo('vertices', vertContent)                 #cria um novo atributo chamado 'vertices'

        edgeContent = graphString.split('A = ')[1].split(';')[0].strip()
        edgeContent = edgeContent.strip("{}")
        edgeArray = []
        for x in edgeContent.split('),('): #divide edgeContent por '),('
            x = x.strip("()")
            u, v = x.split(',')            #guarda cada membro da aresta em u e v
            edgeArray.append((u,v))        #guarda u e v na lista

        self.adicionar_atributo('arestas', edgeArray) #cria um novo atributo chamado 'arestas'

