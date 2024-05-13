import re

class grafo:
    def pattern(string):
        test = r"^V\s*=\s*{[0-9]?[,0-9]*};\s*A\s*=\s*{[\(0-9,0-9\)]?[,\(0-9,0-9\)]*}$"#regex mágica do grafooooo
        result = re.match(test, string)
        return bool(result)