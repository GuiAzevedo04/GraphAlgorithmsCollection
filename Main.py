from Graph import *

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
