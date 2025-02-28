"""
CSCI 355 Spring 2025
M01
Assignment 1: A* Search on 8 Puzzle (*SOLO*)
Author: Alysar Tabet
Professor: Susn Gass
February 28 2025
Puzzle Class
"""
import heapq

class Puzzle:
    def __init__(self, boardState, parent, move, depth: int, heuristic):
        self.boardState = boardState
        self.parent = parent
        self.move = move  # Move taken to reach this state
        self.depth = depth  # "g" in the A* algorithm
        self.heuristic = heuristic  # Manhattan distance
        self.cost = self.depth + self.heuristic  # f (cost) = g (depth) 
    
    def __lt__(self, other):  # Priority queue sorting
        return self.cost < other.cost
    
    def printBoard(self):
        for i in range(3):
            print(self.boardState[i])

    def isGoalState(self) -> bool:
        return self.boardState == FINAL_BOARD_STATE

def heuristicOfState(state):
    """Computes Manhattan distance heuristic."""
    goal_positions = {
        "1": (0, 0), "2": (0, 1), "3": (0, 2),
        "8": (1, 0), "X": (1, 1), "4": (1, 2),
        "7": (2, 0), "6": (2, 1), "5": (2, 2)
    }
    distance = 0
    for i in range(3):
        for j in range(3):
            value = state[i][j]
            if value != "X":
                goal_x, goal_y = goal_positions[value]
                distance += abs(i - goal_x) + abs(j - goal_y)
    return distance

def get_neighbors(state):
    """Returns possible moves from the current state."""
    neighbors = []
    for i in range(3):
        for j in range(3):
            if state[i][j] == "X":
                x, y = i, j
    moves = {
        'Up': (x - 1, y), 'Down': (x + 1, y),
        'Left': (x, y - 1), 'Right': (x, y + 1)
    }
    
    for move, (new_x, new_y) in moves.items():
        if 0 <= new_x < 3 and 0 <= new_y < 3:
            new_state = [list(row) for row in state]  # Copy state
            new_state[x][y], new_state[new_x][new_y] = new_state[new_x][new_y], new_state[x][y]  # Swap tiles
            neighbors.append((tuple(map(tuple, new_state)), move))
    
    return neighbors

FINAL_BOARD_STATE = (
    ("1", "2", "3"),
    ("8", "X", "4"),
    ("7", "6", "5")
)

def solve_puzzle(initial_state):
    """Solves the 8-puzzle using A* search algorithm."""
    open_set = []
    heapq.heappush(open_set, Puzzle(initial_state, None, None, 0, heuristicOfState(initial_state)))
    visited = set()
    
    while open_set:
        current_node = heapq.heappop(open_set)
        visited.add(current_node.boardState)
        
        if current_node.isGoalState():
            return reconstruct_path(current_node)
        
        for neighbor, move in get_neighbors(current_node.boardState):
            if neighbor in visited:
                continue
            new_cost = current_node.depth + 1
            heuristic_cost = heuristicOfState(neighbor)
            heapq.heappush(open_set, Puzzle(neighbor, current_node, move, new_cost, heuristic_cost))
    
    return None  # No solution found

def reconstruct_path(node):
    """Backtracks from the goal state to reconstruct the path taken."""
    path = []
    while node.parent is not None:
        path.append(node.move)
        node = node.parent
    return path[::-1]

if __name__ == "__main__":
    initial_state = (
        ("2", "8", "3"),
        ("1", "6", "4"),
        ("7", "X", "5")
    )  # Example initial state
    solution = solve_puzzle(initial_state)
    
    if solution:
        print("Solution found in", len(solution), "moves:")
        print(" -> ".join(solution))
    else:
        print("No solution found!")
