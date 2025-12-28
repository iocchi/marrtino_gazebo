import random
import argparse
from collections import deque

class MazeGenerator:
    def __init__(self, width, height, seed=None):
        # Definiamo i caratteri grafici
        self.WALL = '█'
        self.PATH = ' '
        self.SOLUTION = '.'
        
        # Dimensioni dispari per garantire la struttura a griglia
        self.width = width if width % 2 != 0 else width + 1
        self.height = height if height % 2 != 0 else height + 1
        
        if seed is not None:
            random.seed(seed)
            self.seed = seed
        else:
            self.seed = "Casuale"
        
        # Inizializza con muri solidi
        self.grid = [[self.WALL for _ in range(self.width)] for _ in range(self.height)]
        self.visited = set()
        self.distances = {}
        self.start_pos = (1, 0)
        self.end_pos = None

    def generate(self):
        """Genera il labirinto usando Recursive Backtracking."""
        self._recursive_backtrack(1, 1, 0)
        
        # Imposta Entrata (S)
        self.grid[0][1] = 'S'
        
        # Trova la cella più lontana per l'Uscita (E) per massimizzare la difficoltà
        farthest_cell = max(self.distances, key=self.distances.get)
        fx, fy = farthest_cell
        
        # Determina la posizione del carattere 'E' sul bordo esterno
        if fy == 1: self.end_pos = (fx, 0)
        elif fy == self.height - 2: self.end_pos = (fx, self.height - 1)
        elif fx == 1: self.end_pos = (0, fy)
        else: self.end_pos = (self.width - 1, fy)
        
        ex, ey = self.end_pos
        self.grid[ey][ex] = 'E'

    def _recursive_backtrack(self, x, y, dist):
        self.grid[y][x] = self.PATH
        self.visited.add((x, y))
        self.distances[(x, y)] = dist
        
        directions = [(0, 2), (0, -2), (2, 0), (-2, 0)]
        random.shuffle(directions)
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 < nx < self.width and 0 < ny < self.height and (nx, ny) not in self.visited:
                # Rimuove il muro tra le celle
                self.grid[y + dy // 2][x + dx // 2] = self.PATH
                self._recursive_backtrack(nx, ny, dist + 1)

    def solve(self):
        """Trova il percorso da S a E usando BFS."""
        start = (1, 0)
        queue = deque([start])
        parent = {start: None}
        
        while queue:
            curr = queue.popleft()
            if curr == self.end_pos:
                break
            
            x, y = curr
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    # Può passare se è un sentiero vuoto o la destinazione
                    if (self.grid[ny][nx] in (self.PATH, 'E')) and (nx, ny) not in parent:
                        parent[(nx, ny)] = curr
                        queue.append((nx, ny))
        
        path = []
        curr = self.end_pos
        while curr is not None:
            path.append(curr)
            curr = parent.get(curr)
        return path

    def display(self, show_solution=False):
        """Stampa il labirinto a video."""
        path_coords = set()
        if show_solution:
            path_coords = set(self.solve())

        title = "RISOLTO" if show_solution else "SFIDA"
        print(f"\n--- LABIRINTO {title} (Seed: {self.seed}) ---")
        
        for y in range(self.height):
            line = ""
            for x in range(self.width):
                char = self.grid[y][x]
                if show_solution and (x, y) in path_coords and char == self.PATH:
                    line += self.SOLUTION
                else:
                    line += char
            print(line)

# --- ESECUZIONE ---
if __name__ == "__main__":

    # Configurazione Argomenti Linea di Comando
    parser = argparse.ArgumentParser(description="Generatore di labirinti ASCII casuali.")
    
    # Argomenti posizionali
    parser.add_argument("width", type=int, help="Larghezza del labirinto (es. 20)")
    parser.add_argument("height", type=int, help="Altezza del labirinto (es. 10)")
    parser.add_argument("seed", type=int, nargs='?', default=None, help="Seed per la generazione casuale (opzionale)")

    args = parser.parse_args()

    w = args.width
    h = args.height
    seed = args.seed
    
    # Parametri: (Larghezza, Altezza, Seed)
    # Nota: la larghezza e altezza visiva dipendono dal font del terminale
    maze = MazeGenerator(w, h, seed)
    maze.generate()
    
    # 1. Stampa labirinto per l'utente
    maze.display(show_solution=False)
    
    # 2. Stampa soluzione
    maze.display(show_solution=True)
