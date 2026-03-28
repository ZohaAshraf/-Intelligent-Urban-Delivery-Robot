# 🤖 Intelligent Urban Delivery Robot
<img width="1920" height="600" alt="performance_comparison" src="https://github.com/user-attachments/assets/dfdd0057-0ab1-4179-8f1f-b2be518a711f" />


> AL-2002 Artificial Intelligence Project 2026 — National University FAST, Faisalabad-Chiniot Campus

A Python simulation of an intelligent delivery robot navigating a 15×15 city grid using **5 AI search algorithms**. The robot avoids buildings, manages traffic zones, and completes 5 sequential deliveries while comparing algorithm performance in real time.

---

## 📸 Preview

| Grid Visualization | Performance Comparison |
|---|---|
| Robot path drawn on city grid | Bar charts comparing all algorithms |
| Buildings, roads, traffic zones | Cost, time, nodes explored |

> Run the script and check your folder — PNG files are generated automatically!

---

## 🗺️ Environment

The city is modeled as a **15×15 grid** with four cell types:

| Cell Type | Symbol | Traversal Cost |
|---|---|---|
| 🟡 Road | Normal path | 1 – 5 (random) |
| ⚫ Building | Obstacle | ❌ Impassable |
| 🟠 Traffic Zone | Congested road | 10 – 20 (random) |
| 🟢 Delivery Point | Package destination | — |
| 🔵 Base Station | Robot start position | (0, 0) |

---

## 🧠 Algorithms Implemented

| Algorithm | Type | Optimal? | Uses Cost? | Uses Heuristic? |
|---|---|---|---|---|
| BFS | Uninformed | ❌ (steps only) | ❌ | ❌ |
| DFS | Uninformed | ❌ | ❌ | ❌ |
| UCS | Uninformed | ✅ | ✅ | ❌ |
| Greedy Best First | Informed | ❌ | ❌ | ✅ |
| A* | Informed | ✅ | ✅ | ✅ |

**Heuristic used:** Manhattan Distance — `|r1 - r2| + |c1 - c2|`

---

## 📦 Features

- ✅ 15×15 grid with buildings, traffic zones, and road cells
- ✅ Random traversal costs for realistic simulation
- ✅ 5 random delivery destinations generated per run
- ✅ Robot position updates after each delivery (chained navigation)
- ✅ All 5 algorithms run for every delivery and compared
- ✅ Performance table printed in terminal for each delivery
- ✅ Grid path visualizations saved as PNG files
- ✅ Bar chart comparison of all algorithms saved as PNG

---

## 🚀 Getting Started

### Prerequisites

```bash
pip install matplotlib numpy
```

### Run

```bash
python delivery_robot.py
```

### Output

After running, the following files will appear in your folder:

```
delivery_1_Astar.png          ← A* path for delivery 1
delivery_1_BFS.png            ← BFS path for delivery 1
delivery_2_Astar.png          ← A* path for delivery 2
delivery_2_BFS.png            ← BFS path for delivery 2
...
performance_comparison.png    ← Bar chart comparing all algorithms
```

---

## 📊 Sample Results

### Terminal Output (per delivery)

```
======================================================================
  DELIVERY 1: From (0, 0) --> To (13, 7)
======================================================================
  Algorithm     Path Cost     Time (s)   Nodes Explored
  ----------------------------------------------------
  BFS                  95     0.000173              150
  DFS                 346     0.000112              111
  UCS                  56     0.000221              150
  Greedy              106     0.000067               26
  Astar                56     0.000180              111
======================================================================
```

### Overall Summary (5 deliveries)

| Algorithm | Total Cost | Total Time (s) | Total Nodes |
|---|---|---|---|
| BFS | 294 | 0.000707 | 443 |
| DFS | 1500 | 0.000477 | 364 |
| UCS | 169 | 0.001062 | 428 |
| Greedy | 285 | 0.000153 | 71 |
| **A*** | **169** | **0.000637** | **361** |

**Key Observations:**
- A* and UCS find the **lowest cost paths** (optimal)
- Greedy is the **fastest** and explores the **fewest nodes**
- DFS produces the **most expensive** paths by far
- BFS finds shortest step-count paths but ignores costs

---

## 🗂️ Code Structure

```
delivery_robot.py
│
├── create_grid()               # Build 15x15 urban environment
├── get_neighbors()             # Get valid adjacent cells
│
├── bfs_search()                # Breadth First Search
├── dfs_search()                # Depth First Search
├── ucs_search()                # Uniform Cost Search
├── greedy_search()             # Greedy Best First Search
├── astar_search()              # A* Search
│
├── manhattan_distance()        # Heuristic function
├── euclidean_distance()        # Alternative heuristic
│
├── run_all_algorithms()        # Run & time all 5 algorithms
├── visualize_grid()            # Save path PNG image
├── plot_performance_comparison() # Save bar chart PNG
└── run_simulation()            # Main driver function
```

---

## 📋 Performance Metrics

Each algorithm is evaluated on:

1. **Path Optimality** — Total traversal cost of the found path
2. **Execution Time** — Seconds taken to compute the path
3. **Search Efficiency** — Number of nodes explored during search

---

## 🛠️ Built With

- **Python 3.x**
- **matplotlib** — Grid and chart visualization
- **heapq** — Priority queue for UCS, Greedy, A*
- **collections.deque** — Queue for BFS
- **math** — Euclidean distance calculation

---

## 📚 Concepts Covered

- Grid-based environment modeling
- Uninformed vs Informed search
- Admissible heuristics (Manhattan distance)
- Priority queues and search frontiers
- Path reconstruction using parent pointers
- Algorithm performance benchmarking

---

## 👤 Author

**[Your Name]**
National University of Computer & Emerging Sciences
Faisalabad-Chiniot Campus | BS Artificial Intelligence
AL-2002 — Artificial Intelligence | Spring 2026

---

## 📄 License

This project was developed for academic purposes as part of AL-2002 coursework.

---

*If this helped you understand search algorithms, give it a ⭐ on GitHub!*
