"""
=============================================================
AL-2002 PROJECT 2026 - MODULE 1
Intelligent Urban Delivery Robot - Path Planning System
=============================================================
Description:
    This program simulates an intelligent delivery robot navigating
    a 15x15 grid-based urban environment. It implements 5 search
    algorithms (BFS, DFS, UCS, Greedy Best First, A*) to find
    optimal paths for delivering packages to 5 random locations.
    Performance of each algorithm is compared based on path cost,
    execution time, and number of nodes explored.
=============================================================
"""

import random
import time
import heapq
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from collections import deque

# ---------------------------------------------------------------
# GRID CELL TYPE CONSTANTS
# ---------------------------------------------------------------
ROAD = 0         # normal road cell
BUILDING = 1     # obstacle - cannot be traversed
TRAFFIC = 2      # traffic zone - higher cost
DELIVERY = 3     # delivery destination
BASE = 4         # starting base station

# ---------------------------------------------------------------
# COLOR MAP for visualization
# ---------------------------------------------------------------
CELL_COLORS = {
    ROAD:     'lightyellow',
    BUILDING: 'dimgray',
    TRAFFIC:  'lightsalmon',
    DELIVERY: 'limegreen',
    BASE:     'dodgerblue'
}

GRID_SIZE = 15   # 15x15 grid


# ---------------------------------------------------------------
# FUNCTION: create_grid
# Creates the 15x15 urban environment grid with buildings,
# roads, traffic zones, and assigns random costs to each cell.
# ---------------------------------------------------------------
def create_grid(base_station, delivery_locations):
    """
    Builds the city grid and assigns cell types and traversal costs.
    Returns:
        grid      -> 2D list of cell types
        cost_grid -> 2D list of traversal costs per cell
    """
    grid = [[ROAD] * GRID_SIZE for _ in range(GRID_SIZE)]
    cost_grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]

    # Place buildings (obstacles) - fixed positions to ensure solvability
    building_cells = [
        (1,1),(1,2),(2,1),(2,2),
        (1,6),(1,7),(2,6),(2,7),
        (4,10),(4,11),(5,10),(5,11),
        (7,3),(7,4),(8,3),(8,4),
        (7,8),(7,9),(8,8),(8,9),
        (10,1),(10,2),(11,1),(11,2),
        (10,6),(10,7),(11,6),(11,7),
        (12,12),(12,13),(13,12),(13,13),
        (3,13),(3,14),(4,13),(4,14),
        (9,12),(9,13),(10,12),(10,13),
    ]

    for (row, col) in building_cells:
        grid[row][col] = BUILDING

    # Assign random traversal costs for all road cells
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid[r][c] == ROAD:
                cost_grid[r][c] = random.randint(1, 5)

    # Place traffic zones (higher cost areas)
    traffic_cells = [
        (0,5),(0,6),(1,5),(2,5),
        (5,0),(5,1),(6,0),(6,1),
        (5,7),(5,8),(6,7),(6,8),
        (9,5),(9,6),(10,5),(10,4),
        (13,3),(13,4),(14,3),(14,4),
        (3,9),(3,10),(2,9),(2,10),
        (11,10),(11,11),(12,10),(12,11),
    ]

    for (r, c) in traffic_cells:
        if grid[r][c] != BUILDING:
            grid[r][c] = TRAFFIC
            cost_grid[r][c] = random.randint(10, 20)

    # Mark base station
    grid[base_station[0]][base_station[1]] = BASE
    cost_grid[base_station[0]][base_station[1]] = 1

    # Mark delivery locations (make sure they are not buildings)
    for loc in delivery_locations:
        r, c = loc
        if grid[r][c] != BUILDING:
            grid[r][c] = DELIVERY
            if cost_grid[r][c] == 0:
                cost_grid[r][c] = random.randint(1, 5)

    return grid, cost_grid


# ---------------------------------------------------------------
# FUNCTION: get_neighbors
# Returns valid neighboring cells (up, down, left, right).
# Skips cells that are buildings (obstacles).
# ---------------------------------------------------------------
def get_neighbors(row, col, grid):
    """
    Gets all valid adjacent cells the robot can move to.
    Moves: Up, Down, Left, Right (no diagonal movement).
    """
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for dr, dc in directions:
        new_row = row + dr
        new_col = col + dc
        if 0 <= new_row < GRID_SIZE and 0 <= new_col < GRID_SIZE:
            if grid[new_row][new_col] != BUILDING:
                neighbors.append((new_row, new_col))
    return neighbors


# ---------------------------------------------------------------
# FUNCTION: reconstruct_path
# Traces back from goal to start using the parent dictionary.
# ---------------------------------------------------------------
def reconstruct_path(parent_map, start, goal):
    """
    Rebuilds the path from start to goal using parent pointers.
    Returns a list of (row, col) tuples from start to goal.
    """
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent_map[current]
    path.reverse()
    return path


# ---------------------------------------------------------------
# FUNCTION: calculate_path_cost
# Sums up the traversal costs along the given path.
# ---------------------------------------------------------------
def calculate_path_cost(path, cost_grid):
    """
    Computes total cost of traveling along the found path.
    """
    total_cost = 0
    for (r, c) in path:
        total_cost += cost_grid[r][c]
    return total_cost


# ---------------------------------------------------------------
# FUNCTION: bfs_search
# Breadth First Search - explores nodes level by level.
# Does NOT consider traversal costs (unweighted).
# ---------------------------------------------------------------
def bfs_search(grid, cost_grid, start, goal):
    """
    BFS: Uses a queue (FIFO). Explores all neighbors at current
    depth before moving deeper. Finds shortest path in terms of
    number of steps, not cost.
    """
    queue = deque()
    queue.append(start)

    visited = set()
    visited.add(start)

    parent_map = {start: None}
    nodes_explored = 0

    while queue:
        current = queue.popleft()
        nodes_explored += 1

        if current == goal:
            path = reconstruct_path(parent_map, start, goal)
            total_cost = calculate_path_cost(path, cost_grid)
            return path, total_cost, nodes_explored

        for neighbor in get_neighbors(current[0], current[1], grid):
            if neighbor not in visited:
                visited.add(neighbor)
                parent_map[neighbor] = current
                queue.append(neighbor)

    return None, float('inf'), nodes_explored  # No path found


# ---------------------------------------------------------------
# FUNCTION: dfs_search
# Depth First Search - explores as deep as possible before backtracking.
# May not find the optimal path but uses less memory than BFS.
# ---------------------------------------------------------------
def dfs_search(grid, cost_grid, start, goal):
    """
    DFS: Uses a stack (LIFO). Goes deep into one branch before
    trying others. Does not guarantee shortest path.
    """
    stack = [start]
    visited = set()
    visited.add(start)

    parent_map = {start: None}
    nodes_explored = 0

    while stack:
        current = stack.pop()
        nodes_explored += 1

        if current == goal:
            path = reconstruct_path(parent_map, start, goal)
            total_cost = calculate_path_cost(path, cost_grid)
            return path, total_cost, nodes_explored

        for neighbor in get_neighbors(current[0], current[1], grid):
            if neighbor not in visited:
                visited.add(neighbor)
                parent_map[neighbor] = current
                stack.append(neighbor)

    return None, float('inf'), nodes_explored


# ---------------------------------------------------------------
# FUNCTION: ucs_search
# Uniform Cost Search - always expands the lowest cost node.
# Guarantees optimal path based on actual traversal costs.
# ---------------------------------------------------------------
def ucs_search(grid, cost_grid, start, goal):
    """
    UCS: Uses a priority queue ordered by cumulative path cost.
    Always picks the cheapest unvisited node. Finds optimal path.
    """
    # Priority queue: (cumulative_cost, node)
    priority_queue = []
    heapq.heappush(priority_queue, (0, start))

    visited = set()
    parent_map = {start: None}
    cost_so_far = {start: 0}
    nodes_explored = 0

    while priority_queue:
        current_cost, current = heapq.heappop(priority_queue)

        if current in visited:
            continue

        visited.add(current)
        nodes_explored += 1

        if current == goal:
            path = reconstruct_path(parent_map, start, goal)
            return path, current_cost, nodes_explored

        for neighbor in get_neighbors(current[0], current[1], grid):
            new_cost = current_cost + cost_grid[neighbor[0]][neighbor[1]]
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                parent_map[neighbor] = current
                heapq.heappush(priority_queue, (new_cost, neighbor))

    return None, float('inf'), nodes_explored


# ---------------------------------------------------------------
# FUNCTION: manhattan_distance
# Heuristic function used by Greedy and A* search.
# Estimates distance between two grid cells.
# ---------------------------------------------------------------
def manhattan_distance(cell_a, cell_b):
    """
    Manhattan distance heuristic: |row1 - row2| + |col1 - col2|
    Used to guide informed search algorithms toward the goal.
    """
    return abs(cell_a[0] - cell_b[0]) + abs(cell_a[1] - cell_b[1])


# ---------------------------------------------------------------
# FUNCTION: euclidean_distance
# Alternative heuristic using straight-line distance.
# ---------------------------------------------------------------
def euclidean_distance(cell_a, cell_b):
    """
    Euclidean distance heuristic: sqrt((r1-r2)^2 + (c1-c2)^2)
    """
    return math.sqrt((cell_a[0] - cell_b[0])**2 + (cell_a[1] - cell_b[1])**2)


# ---------------------------------------------------------------
# FUNCTION: greedy_search
# Greedy Best First Search - expands node closest to goal.
# Uses only heuristic, ignores actual path cost.
# ---------------------------------------------------------------
def greedy_search(grid, cost_grid, start, goal):
    """
    Greedy: Uses priority queue ordered by heuristic only h(n).
    Fast but not guaranteed to find optimal path.
    Uses Manhattan distance as the heuristic.
    """
    priority_queue = []
    h = manhattan_distance(start, goal)
    heapq.heappush(priority_queue, (h, start))

    visited = set()
    parent_map = {start: None}
    nodes_explored = 0

    while priority_queue:
        _, current = heapq.heappop(priority_queue)

        if current in visited:
            continue

        visited.add(current)
        nodes_explored += 1

        if current == goal:
            path = reconstruct_path(parent_map, start, goal)
            total_cost = calculate_path_cost(path, cost_grid)
            return path, total_cost, nodes_explored

        for neighbor in get_neighbors(current[0], current[1], grid):
            if neighbor not in visited:
                parent_map[neighbor] = current
                h = manhattan_distance(neighbor, goal)
                heapq.heappush(priority_queue, (h, neighbor))

    return None, float('inf'), nodes_explored


# ---------------------------------------------------------------
# FUNCTION: astar_search
# A* Search - combines actual cost g(n) and heuristic h(n).
# Most efficient informed search; finds optimal path.
# ---------------------------------------------------------------
def astar_search(grid, cost_grid, start, goal):
    """
    A*: Uses f(n) = g(n) + h(n) as priority.
    g(n) = cost from start to current node
    h(n) = estimated cost from current node to goal (Manhattan)
    Finds the optimal path efficiently.
    """
    priority_queue = []
    g_start = 0
    h_start = manhattan_distance(start, goal)
    heapq.heappush(priority_queue, (g_start + h_start, start))

    visited = set()
    parent_map = {start: None}
    g_cost = {start: 0}
    nodes_explored = 0

    while priority_queue:
        f_val, current = heapq.heappop(priority_queue)

        if current in visited:
            continue

        visited.add(current)
        nodes_explored += 1

        if current == goal:
            path = reconstruct_path(parent_map, start, goal)
            return path, g_cost[current], nodes_explored

        for neighbor in get_neighbors(current[0], current[1], grid):
            new_g = g_cost[current] + cost_grid[neighbor[0]][neighbor[1]]
            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                parent_map[neighbor] = current
                h = manhattan_distance(neighbor, goal)
                f = new_g + h
                heapq.heappush(priority_queue, (f, neighbor))

    return None, float('inf'), nodes_explored


# ---------------------------------------------------------------
# FUNCTION: run_all_algorithms
# Runs all 5 search algorithms for a given start and goal.
# Records path, cost, time, and nodes explored for each.
# ---------------------------------------------------------------
def run_all_algorithms(grid, cost_grid, start, goal):
    """
    Runs BFS, DFS, UCS, Greedy, and A* from start to goal.
    Returns a dictionary with results for each algorithm.
    """
    algorithms = {
        'BFS':    bfs_search,
        'DFS':    dfs_search,
        'UCS':    ucs_search,
        'Greedy': greedy_search,
        'Astar':  astar_search
    }

    results = {}

    for algo_name, algo_function in algorithms.items():
        start_time = time.time()
        path, cost, nodes = algo_function(grid, cost_grid, start, goal)
        end_time = time.time()

        elapsed_time = round(end_time - start_time, 6)

        results[algo_name] = {
            'path':           path,
            'cost':           cost,
            'time':           elapsed_time,
            'nodes_explored': nodes
        }

    return results


# ---------------------------------------------------------------
# FUNCTION: visualize_grid
# Draws the grid and paths using matplotlib.
# Shows the robot's movement and delivery locations.
# ---------------------------------------------------------------
def visualize_grid(grid, path, start, goal, delivery_number, algo_name):
    """
    Creates a visual representation of the grid and planned path.
    Different colors represent different cell types.
    The robot's path is shown as a blue line on the grid.
    """
    fig, ax = plt.subplots(figsize=(9, 9))

    # Draw each cell with its corresponding color
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            cell_type = grid[r][c]
            color = CELL_COLORS.get(cell_type, 'white')
            rect = plt.Rectangle((c, GRID_SIZE - 1 - r), 1, 1,
                                  facecolor=color, edgecolor='gray', linewidth=0.5)
            ax.add_patch(rect)

    # Draw the planned path
    if path:
        path_rows = [GRID_SIZE - 1 - r for (r, c) in path]
        path_cols = [c + 0.5 for (r, c) in path]
        path_rows_centered = [v + 0.5 for v in path_rows]
        ax.plot(path_cols, path_rows_centered, color='blue',
                linewidth=2.5, zorder=5, label='Path')

    # Mark start (base station)
    ax.text(start[1] + 0.5, GRID_SIZE - 1 - start[0] + 0.5,
            'S', ha='center', va='center',
            fontsize=11, fontweight='bold', color='white', zorder=6)

    # Mark goal (delivery location)
    ax.text(goal[1] + 0.5, GRID_SIZE - 1 - goal[0] + 0.5,
            'G', ha='center', va='center',
            fontsize=11, fontweight='bold', color='black', zorder=6)

    # Create legend
    legend_items = [
        mpatches.Patch(color='lightyellow', label='Road'),
        mpatches.Patch(color='dimgray',     label='Building'),
        mpatches.Patch(color='lightsalmon', label='Traffic Zone'),
        mpatches.Patch(color='limegreen',   label='Delivery Point'),
        mpatches.Patch(color='dodgerblue',  label='Base Station'),
        mpatches.Patch(color='blue',        label='Robot Path'),
    ]
    ax.legend(handles=legend_items, loc='upper right',
              fontsize=7, framealpha=0.9)

    ax.set_xlim(0, GRID_SIZE)
    ax.set_ylim(0, GRID_SIZE)
    ax.set_xticks(range(GRID_SIZE))
    ax.set_yticks(range(GRID_SIZE))
    ax.set_xticklabels(range(GRID_SIZE), fontsize=7)
    ax.set_yticklabels(range(GRID_SIZE - 1, -1, -1), fontsize=7)
    ax.set_title(f'Delivery {delivery_number} | Algorithm: {algo_name} | '
                 f'Start: {start} -> Goal: {goal}', fontsize=10)
    ax.set_xlabel('Column')
    ax.set_ylabel('Row')
    ax.grid(True, color='gray', linewidth=0.3)

    plt.tight_layout()
    plt.savefig(f'delivery_{delivery_number}_{algo_name}.png', dpi=100)
    plt.close()


# ---------------------------------------------------------------
# FUNCTION: print_performance_table
# Prints a formatted comparison table for all algorithms.
# ---------------------------------------------------------------
def print_performance_table(results, delivery_num, start, goal):
    """
    Displays a formatted performance comparison table for one delivery.
    Shows cost, time, and nodes explored for each algorithm.
    """
    print(f"\n{'='*70}")
    print(f"  DELIVERY {delivery_num}: From {start} --> To {goal}")
    print(f"{'='*70}")
    print(f"  {'Algorithm':<10} {'Path Cost':>12} {'Time (s)':>12} {'Nodes Explored':>16}")
    print(f"  {'-'*52}")
    for algo_name, data in results.items():
        cost = data['cost'] if data['path'] else 'No Path'
        cost_str = str(cost) if isinstance(cost, str) else f"{cost}"
        print(f"  {algo_name:<10} {cost_str:>12} {data['time']:>12.6f} {data['nodes_explored']:>16}")
    print(f"{'='*70}")


# ---------------------------------------------------------------
# FUNCTION: plot_performance_comparison
# Creates bar charts comparing algorithms across all deliveries.
# ---------------------------------------------------------------
def plot_performance_comparison(all_results):
    """
    Plots bar graphs comparing path cost, execution time, and
    nodes explored across all 5 algorithms for all deliveries.
    """
    algo_names = ['BFS', 'DFS', 'UCS', 'Greedy', 'Astar']
    colors = ['steelblue', 'tomato', 'seagreen', 'orchid', 'goldenrod']

    # Aggregate average values across all deliveries
    avg_cost   = {a: [] for a in algo_names}
    avg_time   = {a: [] for a in algo_names}
    avg_nodes  = {a: [] for a in algo_names}

    for delivery_results in all_results:
        for algo in algo_names:
            data = delivery_results[algo]
            avg_cost[algo].append(data['cost'] if data['path'] else 0)
            avg_time[algo].append(data['time'])
            avg_nodes[algo].append(data['nodes_explored'])

    # Compute means
    mean_cost  = [sum(avg_cost[a])  / len(avg_cost[a])  for a in algo_names]
    mean_time  = [sum(avg_time[a])  / len(avg_time[a])  for a in algo_names]
    mean_nodes = [sum(avg_nodes[a]) / len(avg_nodes[a]) for a in algo_names]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('Algorithm Performance Comparison (Averaged over 5 Deliveries)',
                 fontsize=13, fontweight='bold')

    # --- Plot 1: Average Path Cost ---
    axes[0].bar(algo_names, mean_cost, color=colors, edgecolor='black')
    axes[0].set_title('Average Path Cost')
    axes[0].set_ylabel('Total Traversal Cost')
    axes[0].set_xlabel('Algorithm')
    for i, val in enumerate(mean_cost):
        axes[0].text(i, val + 0.5, f'{val:.1f}', ha='center', va='bottom', fontsize=9)

    # --- Plot 2: Average Execution Time ---
    axes[1].bar(algo_names, mean_time, color=colors, edgecolor='black')
    axes[1].set_title('Average Execution Time (seconds)')
    axes[1].set_ylabel('Time (s)')
    axes[1].set_xlabel('Algorithm')
    for i, val in enumerate(mean_time):
        axes[1].text(i, val + 0.000001, f'{val:.5f}', ha='center', va='bottom', fontsize=8)

    # --- Plot 3: Average Nodes Explored ---
    axes[2].bar(algo_names, mean_nodes, color=colors, edgecolor='black')
    axes[2].set_title('Average Nodes Explored')
    axes[2].set_ylabel('Number of Nodes')
    axes[2].set_xlabel('Algorithm')
    for i, val in enumerate(mean_nodes):
        axes[2].text(i, val + 0.5, f'{val:.0f}', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plt.savefig('performance_comparison.png', dpi=120)
    plt.close()
    print("\n[INFO] Performance comparison chart saved.")


# ---------------------------------------------------------------
# FUNCTION: run_simulation
# Main simulation driver - runs the full delivery process.
# ---------------------------------------------------------------
def run_simulation():
    """
    Main function to run the entire delivery robot simulation.
    Steps:
        1. Set up the grid environment
        2. Generate 5 random delivery locations
        3. For each delivery, run all 5 algorithms
        4. Visualize each path and compare performance
    """
    random.seed(42)  # Set seed for reproducibility

    # Base station - robot starts here
    base_station = (0, 0)

    # Generate 5 random delivery locations (avoid buildings and base)
    building_cells = [
        (1,1),(1,2),(2,1),(2,2),(1,6),(1,7),(2,6),(2,7),
        (4,10),(4,11),(5,10),(5,11),(7,3),(7,4),(8,3),(8,4),
        (7,8),(7,9),(8,8),(8,9),(10,1),(10,2),(11,1),(11,2),
        (10,6),(10,7),(11,6),(11,7),(12,12),(12,13),(13,12),(13,13),
        (3,13),(3,14),(4,13),(4,14),(9,12),(9,13),(10,12),(10,13),
    ]
    all_possible = [
        (r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE)
        if (r, c) != base_station and (r, c) not in building_cells
    ]
    delivery_locations = random.sample(all_possible, 5)

    print("=" * 70)
    print("   AL-2002 PROJECT 2026 - MODULE 1")
    print("   Intelligent Urban Delivery Robot Simulation")
    print("=" * 70)
    print(f"\n  Base Station: {base_station}")
    print(f"  Delivery Locations: {delivery_locations}")
    print(f"  Grid Size: {GRID_SIZE} x {GRID_SIZE}")

    # Build the grid environment
    grid, cost_grid = create_grid(base_station, delivery_locations)

    all_results = []       # Stores results for all 5 deliveries
    current_start = base_station  # Robot starts from base

    # ---- Run delivery for each destination ----
    for delivery_num, goal in enumerate(delivery_locations, start=1):

        print(f"\n\n>>> DELIVERY {delivery_num}: Robot moving to {goal}")
        print(f"    Current Start: {current_start}")

        # Skip if goal is a building (shouldn't happen with sample)
        if grid[goal[0]][goal[1]] == BUILDING:
            print(f"    [WARNING] Goal {goal} is a building. Skipping.")
            continue

        # Run all 5 algorithms
        results = run_all_algorithms(grid, cost_grid, current_start, goal)
        all_results.append(results)

        # Print performance table
        print_performance_table(results, delivery_num, current_start, goal)

        # Visualize path for the best algorithm (A*) and also BFS
        for algo_name in ['Astar', 'BFS']:
            if results[algo_name]['path']:
                visualize_grid(grid, results[algo_name]['path'],
                               current_start, goal, delivery_num, algo_name)
                print(f"    [INFO] {algo_name} path visualization saved.")

        # After delivery, robot stays at the goal for next task
        current_start = goal

    # ---- Overall Performance Summary ----
    print("\n\n" + "=" * 70)
    print("   OVERALL PERFORMANCE SUMMARY (All 5 Deliveries)")
    print("=" * 70)

    algo_names = ['BFS', 'DFS', 'UCS', 'Greedy', 'Astar']
    for algo in algo_names:
        total_cost  = sum(r[algo]['cost']           for r in all_results if r[algo]['path'])
        total_time  = sum(r[algo]['time']            for r in all_results)
        total_nodes = sum(r[algo]['nodes_explored']  for r in all_results)
        print(f"\n  {algo}:")
        print(f"    Total Path Cost    : {total_cost}")
        print(f"    Total Time (s)     : {total_time:.6f}")
        print(f"    Total Nodes Explored: {total_nodes}")

    # ---- Generate Performance Comparison Charts ----
    plot_performance_comparison(all_results)

    print("\n\n  [SIMULATION COMPLETE]")
    print("  All delivery tasks finished successfully.\n")


# ---------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------
if __name__ == "__main__":
    run_simulation()