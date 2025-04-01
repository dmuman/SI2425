from searchPlus import *
from collections import namedtuple


line1 = "## ## ## ## ## ## ## ## ##\n"
line2 = "## .. .. .. .. .. .. .. ##\n"
line3 = "## .. .. .. 00 .. .. .. ##\n"
line4 = "## .. .. .. .. .. .. .. ##\n"
line5 = "## .. .. () () () .. .. ##\n"
line6 = "## .. .. .. .. .. .. .. ##\n"
line7 = "## .. .. .. 01 .. .. .. ##\n"
line8 = "## .. .. .. .. .. .. .. ##\n"
line9 = "## ## ## ## ## ## ## ## ##\n"
grid = line1 + line2 + line3 + line4 + line5 + line6 + line7 + line8 + line9


EstadoPin = namedtuple('EstadoPin','pinguins')

class EstadoPinguins(EstadoPin):
        
    def __hash__(self):
        return hash(tuple(sorted(self.pinguins.items())))
    
    def __lt__(self,other):
        return True
    
    
class PenguinsPairs(Problem):
    
    def process_txt(self, grid):
        """
        Função que porcessa a grelha em txt e obtém as posições dos icebergues (walls), água e pinguins.
        O output desta função é um dicionário em que as chaves identifica cada tipo de informação a ser 
        guardada. Os pinguins serão um dicionário e os icebergues e água serão um conjunto.
        """
        data = {'walls': set(), 'pinguins': {}, 'water': set()}
        lines = grid.split('\n')[:-1]
        x = 0
        for row in lines:
            ll = row.split()
            y = 0
            for col in ll:
                if col == '##':
                    data['walls'].add((x,y))
                elif col == '()':
                    data['water'].add((x,y))
                elif col != '..':
                    data['pinguins'][col] = (x,y)
                y += 1
            x += 1
        data['dim'] = (len(lines),len(ll))
        return data
    
    
    directions = {"N":(-1, 0), "E":(0, +1), "S":(+1, 0), "O":(0, -1)}  # ortogonais
    
    
    def __init__(self, ice_map=grid):
        initialStatus = self.process_txt(ice_map) # processa o txt e converte num dicionário
        self.initial = EstadoPinguins(initialStatus['pinguins']) # estado inicial
        self.goal = {} # estado vazio
        self.obstacles = initialStatus['walls'] # posições do icebergues
        self.water = initialStatus['water'] # posições da água
        self.dim = initialStatus['dim'] # dimensão do mapa (não precisa de ser quadrado)

    
    def slide(self,state,x,y,d):
        """
        Função que identifica se é possível o pinguim se deslocar numa direção.
        """
        (dx,dy) = self.directions[d]
        if (x+dx,y+dy) in self.obstacles:
            return False
        while (x+dx,y+dy) not in self.obstacles and (x+dx,y+dy) not in state.pinguins.values():
            if (x+dx,y+dy) in self.water:
                return False
            x = x+dx
            y = y+dy
        return True
    
    
    def actions (self, state):
        """
        Devolve uma lista ordenada com todas as ações possíveis para o estado.
        """
        actions_list = []
        for p in state.pinguins.keys():
            x,y = state.pinguins[p] # coordenadas da posição do pinguim 
            if self.slide(state,x,y,"N"): # verifica se o pinguim pode deslocar-se para Norte
                actions_list.append((p,"N"))
            if self.slide(state,x,y,"S"):
                actions_list.append((p,"S"))
            if self.slide(state,x,y,"E"):
                actions_list.append((p,"E"))
            if self.slide(state,x,y,"O"):
                actions_list.append((p,"O"))
        return sorted(actions_list) 
    

    def result (self, state, action):
        """
        Devolve um novo estado resultante de aplicar "action" a "state".
        """
        p,d = action # ação é um tuplo (ID pinguim, direção)
        pinguins = state.pinguins.copy()
        x,y = state.pinguins[p]
        (dx,dy) = self.directions[d]
        # desloca o pinguim até que embata num obstáculo ou noutro pinguim
        while (x+dx,y+dy) not in self.obstacles and (x+dx,y+dy) not in state.pinguins.values():
            x = x+dx
            y = y+dy
        # se o pinguim embateu num obstáculo, atualiza-se a posição do pinguim
        if (x+dx,y+dy) in self.obstacles:
            pinguins[p] = (x,y)
        # se o pinguim embateu noutro pinguim, ambos são removidos do estado
        if (x+dx,y+dy) in state.pinguins.values():
            pinguins.pop(p)
            for key in state.pinguins:
                if state.pinguins[key] == (x+dx,y+dy):
                    pinguins.pop(key)
                    break
        return EstadoPinguins(pinguins)
    

    def goal_test (self, state):
        """
        Verifica se todos os pinguins estão emparelhados, ou seja, se os estado está vazio.
        """
        return state.pinguins == {}
    

    def display (self, state):
        """
        Devolve a grelha em modo txt.
        """
        output = ""
        for i in range(self.dim[0]):
            for j in range(self.dim[1]):
                if (i,j) in state.pinguins.values():
                    for key in state.pinguins:
                        if state.pinguins[key] == (i,j):
                            ch = key
                            break
                elif (i,j) in self.obstacles:
                    ch = "##"
                elif (i,j) in self.water:
                    ch = "()"
                else:
                    ch = ".."
                output += ch + " "
            output += "\n"
        return output
    

    def executa(self, state, actions_list, verbose=False):
        """Executa uma sequência de acções a partir do estado devolvendo o triplo formado pelo estado, 
        pelo custo acumulado e pelo booleano que indica se o objectivo foi ou não atingido. Se o objectivo 
        for atingido antes da sequência ser atingida, devolve-se o estado e o custo corrente.
        Há o modo verboso e o não verboso, por defeito."""
        cost = 0
        for a in actions_list:
            seg = self.result(state,a)
            cost = self.path_cost(cost,state,a,seg)
            state = seg
            obj = self.goal_test(state)
            if verbose:
                print('Ação:', a)
                print(self.display(state),end='')
                print('Custo Total:',cost)
                print('Atingido o objectivo?', obj)
                print()
            if obj:
                break
        return (state, cost, obj)
    
    
    def halfPenguins(self, node):
        """
        Heurística que considera metade do número de pinguins.
        """
        return len(node.state.pinguins)//2
        

    def Npairings(self, node):
        """
        Heurística que conta o número máximo de pares de pinguins que podem ser formados
        em linhas retas horizontais ou verticais sem obstruções.
        """
        # ordena os pinguins por ID para garantir consistência
        penguins = sorted(node.state.pinguins.items())
        paired = set()  # conjunto de pinguins já emparelhados
        np = 0  # contador de pares formados

        # percorre todos os pinguins
        for i, (p1, pos1) in enumerate(penguins):
            if p1 in paired:
                continue  # pinguim já emparelhado
            
            x1, y1 = pos1
            best_pair = None

            # procura um par para o pinguim atual
            for j, (p2, pos2) in enumerate(penguins):
                if i >= j or p2 in paired:
                    continue  # não verificar com ele mesmo ou com pinguins já emparelhados

                x2, y2 = pos2

                # verifica alinhamento horizontal
                if x1 == x2:
                    y_min, y_max = sorted([y1, y2])
                    path_clear = True
                    # verifica cada posição no caminho
                    for y in range(y_min + 1, y_max):
                        if (x1, y) in self.obstacles or (x1, y) in self.water:
                            path_clear = False
                            break
                        # verifica se há outros pinguins no caminho
                        if any(pos == (x1, y) for pos in node.state.pinguins.values()):
                            path_clear = False
                            break
                    if path_clear:
                        best_pair = p2
                        break

                # verifica alinhamento vertical
                elif y1 == y2:
                    x_min, x_max = sorted([x1, x2])
                    path_clear = True
                    # verifica cada posição no caminho
                    for x in range(x_min + 1, x_max):
                        if (x, y1) in self.obstacles or (x, y1) in self.water:
                            path_clear = False
                            break
                        # verifica se há outros pinguins no caminho
                        if any(pos == (x, y1) for pos in node.state.pinguins.values()):
                            path_clear = False
                            break
                    if path_clear:
                        best_pair = p2
                        break

            # se encontrou um par válido, marca ambos como emparelhados
            if best_pair:
                paired.add(p1)
                paired.add(best_pair)
                np += 1

        # calcula pinguins não emparelhados e retorna o valor heurístico
        unpaired = len(node.state.pinguins) - 2 * np
        return max(np + unpaired, len(node.state.pinguins) // 2)


    def highestPairings(self, node):
        """
        Heurística gulosa que prioriza emparelhar pinguins com maior número de possibilidades,
        tentando maximizar o número total de pares formados.
        """
        penguin_pos = node.state.pinguins
        penguins = sorted(penguin_pos.keys())  # ordena por ID para consistência
        paired = set()  # conjunto de pinguins já emparelhados
        np = 0  # contador de pares formados

        while True:
            pair_counts = {}  # número de pares possíveis para cada pinguim
            possible_pairs = {}  # lista de pares possíveis para cada pinguim

            # calcula possíveis pares para cada pinguim não emparelhado
            for p1 in penguins:
                if p1 in paired:
                    continue
                
                x1, y1 = penguin_pos[p1]
                possible = []

                # verifica todos os outros pinguins como possíveis pares
                for p2 in penguins:
                    if p1 == p2 or p2 in paired:
                        continue
                    
                    x2, y2 = penguin_pos[p2]

                    # verifica alinhamento horizontal
                    if x1 == x2:
                        y_min, y_max = sorted([y1, y2])
                        path_clear = True
                        for y in range(y_min + 1, y_max):
                            if (x1, y) in self.obstacles or (x1, y) in self.water:
                                path_clear = False
                                break
                            if any(pos == (x1, y) for pos in penguin_pos.values()):
                                path_clear = False
                                break
                        if path_clear:
                            possible.append(p2)
                    
                    # verifica alinhamento vertical
                    elif y1 == y2:
                        x_min, x_max = sorted([x1, x2])
                        path_clear = True
                        for x in range(x_min + 1, x_max):
                            if (x, y1) in self.obstacles or (x, y1) in self.water:
                                path_clear = False
                                break
                            if any(pos == (x, y1) for pos in penguin_pos.values()):
                                path_clear = False
                                break
                        if path_clear:
                            possible.append(p2)

                possible_pairs[p1] = possible
                pair_counts[p1] = len(possible)

            # se não há mais pares possíveis, termina
            if not pair_counts:
                break

            # seleciona o pinguim com mais opções de emparelhamento
            p1 = min(pair_counts.keys(), key=lambda p: (-pair_counts[p], p))
            
            # se não há pares possíveis, termina
            if pair_counts[p1] == 0:
                break

            # seleciona o melhor par disponível
            available_pairs = possible_pairs[p1]
            if available_pairs:
                # escolhe o par que tem mais opções restantes
                p2 = min(available_pairs, key=lambda p: (-pair_counts.get(p, 0), p))
                paired.update([p1, p2])
                np += 1
            else:
                break

        # calcula o valor heurístico final
        total = len(penguin_pos)
        unpaired = total - 2 * np
        return max(np + unpaired, total // 2)