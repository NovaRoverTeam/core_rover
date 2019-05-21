from math import sqrt

def euclidean_heuristic_cost(curr, end):
    """
    Estimates cost from curr (x0,y0) to end (x1,y1) using Euclidean
    distance.
    """
    return sqrt((curr[0]-end[0])**2 + (curr[1]-end[1])**2)

def reconstruct_path_to_destination(prev, end):
    """
    Constructs an in-order sequence of (x,y) coordinates (list of tuples)
    to the end destination using the mapping from nodes to their predecessors
    (prev).
    """
    path = [end]
    curr = end
    while curr in prev.keys():
        curr = prev[curr]
        path.insert(0, curr)
    return path

def edge_distance_between(node_from, node_to, grid):
    """
    The edge distance between any two nodes is the cost of traversing node_from.
    That is, in this context, node_from is actually irrelevant. We'll retain
    the API for this function as is, however, in case our representation of the
    graph changes in future.
    """
    return grid[node_to[0]][node_to[1]]

def neighbors_of(node, grid):
    """
    The neighbors of a cell (node) in the grid are the 8-surrounding cells.
    """
    neighbors = []

    n_rows = len(grid)
    n_cols = len(grid[0])

    for r_delta in [-1, 0, 1]:
        for c_delta in [-1, 0, 1]:
            r = node[0] + r_delta
            c = node[1] + c_delta

            # skip neighbors that would take us off the edge of the grid
            if r < 0 or r >= n_rows or c < 0 or c >= n_cols:
                continue
            # skip the current node itself
            if r_delta == 0 and c_delta == 0:
                continue

            neighbors.append( (r, c) )

    return neighbors

def node_with_min_fscore(open_set, f_cost):
    """
    Find the cell in open set with the smallest f score.
    """
    # TODO: replace this nasty implementation with a heapq
    min_score = float("Inf")
    min_node = None
    for node in open_set:
        if f_cost[node] < min_score:
            min_score = f_cost[node]
            min_node = node
    return min_node

def a_star(grid, start, end, heuristic_cost=euclidean_heuristic_cost):
    """
    Implementation of A Star over a 2D grid. Returns a list of waypoints
    as a list of (x,y) tuples.

    Input:
    : grid, 2D matrix
    : start, (x,y) tuple, start position
    : end, (x,y) tuple, end destination

    Output:
    : waypoints, list of (x,y) tuples
    """
    # the set of cells already evaluated
    closed_set = set()

    # the set of cells already discovered
    open_set = set()
    open_set.add(start)

    # for each cell, mapping to its least-cost incoming cell
    prev = {}
   
    # for each node, cost of reaching it from start (g_cost) 
    # for each node, cost of getting from start to dest via that node (f_cost)
    #   note: cell->dest component of f_cost will be estimated using a heuristic
    g_cost = {}
    f_cost = {}
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            cell = (r, c)
            g_cost[cell] = float("Inf")
            f_cost[cell] = float("Inf")
    g_cost[start] = 0
    f_cost[start] = heuristic_cost(start, end)

    while len(open_set) > 0:
        # node in open set with min fscore
        curr = node_with_min_fscore(open_set, f_cost)
        
        # if we've reached the destination
        if curr == end:
            return reconstruct_path_to_destination(prev, curr)

        open_set.remove(curr)
        closed_set.add(curr)

        for neighbor in neighbors_of(curr, grid):

            # ignore neighbors which have already been evaluated
            if neighbor in closed_set:
                continue

            curr_g_cost = g_cost[curr] + edge_distance_between(curr, neighbor, grid)

            # add neighbor to newly discovered nodes
            if neighbor not in open_set:
                open_set.add(neighbor)

            # if we've already got a lower g_score for neighbor, then move on
            elif curr_g_cost >= g_cost[neighbor]:
                continue

            prev[neighbor] = curr
            g_cost[neighbor] = curr_g_cost
            f_cost[neighbor] = g_cost[neighbor] + heuristic_cost(neighbor, end)

    # if we get to this point, it's not possible to reach the end destination
    return []

if __name__ == "__main__":

    eps = 10e-5

    # test euclidean_heruistic_cost
    test_cases = [
        [(0,0), (1,1), sqrt(2)],
        [(1,2), (8,7), sqrt(49+25)]
    ]
    for tc in test_cases:
        assert(abs(euclidean_heuristic_cost(tc[0], tc[1]) - tc[-1] < eps))

    # test reconstruct_path_to_destination
    test_cases = [
        [
            {(0,1): (0,0), (1,1): (0,1), (1,2): (1,1)},
            (1,2), 
            [(0,0), (0,1), (1,1), (1,2)]
        ],
        [
            {(1,1): (0,1), (2,1): (1,1), (3,1): (2,1)},
            (3,1),
            [(0,1), (1,1), (2,1), (3,1)]
        ]
    ]

    for tc in test_cases:
        assert(reconstruct_path_to_destination(tc[0], tc[1]) == tc[-1])

    # test edge_distance_between
    grid = [
        [0, 0, 10e10, 25],
        [0, 10, 0, 1],
        [0, 0 , 0, 0],
        [1, 2, 10, 0],
        [2, 3, 5, 5]
    ]
    test_cases = [
       [(1,1), (2,2), 0]
    ]

    for tc in test_cases:
        assert(edge_distance_between(tc[0], tc[1], grid) == tc[-1])
    
    # test neighbors_of
    grid = [
        [0, 0, 10e10, 25],
        [0, 10, 0, 1],
        [0, 0 , 0, 0],
        [1, 2, 10, 0],
        [2, 3, 5, 5]
    ]
    test_cases = [
        [(1,1), [(0,1), (1,0), (2,1), (1,2), (0,0), (0,2), (2,2), (2,0)]],
        [(0,0), [(0,1), (1,0), (1,1)]]
    ]

    for tc in test_cases: 
        assert(neighbors_of(tc[0], grid) == sorted(tc[-1]))

    # test node_with_min_fscore
    # start (0, 0), goal (2,2)
    test_cases = [
        [[(0,0)], {(0,0): sqrt(8)}, (0,0)],
        [[(0,1), (1,0), (1,1)], {(0,1): sqrt(5), (1,0): sqrt(5), (1,1): 10 + sqrt(2)}, (0,1)]
    ]

    for tc in test_cases:
        assert(node_with_min_fscore(tc[0], tc[1]) == tc[-1])

    # test a_start
    grid = [
        [0, 0, 10e10, 25],
        [0, 10, 0, 1],
        [0, 0 , 0, 0],
        [1, 2, 10, 0],
        [2, 3, 5, 5]
    ]

    test_cases = [
        [(0,0), (2,2), [(0,0), (0,1), (1,2), (2,2)]],
        [(0,0), (2,1), [(0,0), (1,0), (2,1)]],
        [(0,0), (3,3), [(0,0), (0,1), (1,2), (2,3), (3,3)]]
    ]
    
    for tc in test_cases:
        assert(a_star(grid, tc[0], tc[1]) == tc[-1])