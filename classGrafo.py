import re

class Grafo:
    def __init__(self):
        self.vertices = []
        self.arestas = []
    
    def add_vertice(self, vert):
        if vert not in self.vertices:
            self.vertices.append(vert)

    def add_aresta(self, u, v):
        if (u,v) not in self.aresta:
            self.vertices.append((u,v))

    def adicionar_atributo(self, nome_atributo, valor): #função mágica que adiciona um atributo na classe
        setattr(self, nome_atributo, valor)             #é só adicionar o nome do atributo e seu valor

    def graph_pattern(self, string) -> bool: #identifica se o padrão do txt é válido
        test = r"^\s*V\s*=\s*{[0-9]?[,0-9]*};\s*A\s*=\s*{[\(0-9,0-9\)]?[,\(0-9,0-9\)]*};$"#regex mágica do grafooooo
        result = re.match(test, string)
        return bool(result)
    
    def read_txt(self): #lê o conteúdo do .txt e retorna seu conteúdo
        nameArq = input("insira o nome do arquivo que contém o grafo: ")
        graphArq = open(nameArq, "r")
        content = graphArq.read()

        if(self.graph_pattern(content)):
            return content
        else:
            return None
        
    def read_graph_string(self): #lê a string e produz os vértices e arestas
        try:
            graphString = self.read_txt()

            vertContent = graphString.split('V = ')[1].split(';')[0].strip() #separa em 2 partes usando "V = ", e pega a segunda parte.
            vertContent = vertContent.strip("{}")                            #separa em 2 partes de novo, desta vez com ";", e pega a primeira parte
            self.vertices = list(map(int, vertContent.split(",")))           #por fim, elimina os espaços em branco em volta da string e guarda
        

            edgeContent = graphString.split('A = ')[1].split(';')[0].strip()
            edgeContent = edgeContent.strip("{}")
            edgeArray = []
            for x in edgeContent.split('),('): #divide edgeContent por '),('
                x = x.strip("()")
                u, v = list(map(int, x.split(',')))            #guarda cada membro da aresta em u e v
                edgeArray.append((u,v))        #guarda u e v na lista

            self.arestas = edgeArray #guarda o array em arestas
        except:
            print("O padrão do arquivo está incorreto!")

