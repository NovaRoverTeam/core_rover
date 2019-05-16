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
        path.insert(curr, 0)
    return path

def edge_distance_between(node_from, node_to, grid):
    """
    The edge distance between any two nodes is the cost of traversing node_from.
    That is, in this context, node_from is actually irrelevant. We'll retain
    the API for this function as is, however, in case our representation of the
    graph changes in future.
    """
    return grid[n2[0]][n2[1]]

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
            if r < 0 or r >= nrows or c < 0 or c >= ncols:
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
    max_score = -1
    max_node = Node
    for node in open_set:
        if f_score[node] > max_score:
            max_score = f_score[node]
            max_node = node
    return node

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
            g_cost[cell] = float.inf("inf")
            f_cost[cell] = float.inf("inf")
    g_cost[start] = 0
    f_cost[start] = heuristic_cost(start, end)

    while len(open_set) != 0:
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

            curr_g_score = g_score[curr] + distance_between(curr, neighbor, grid)

            # add neighbor to newly discovered nodes
            if neighbor not in open_set:
                open_set.add(neighbor)

            # if we've already got a lower g_score for neighbor, then move on
            elif curr_g_score >= g_score[neighbor]:
                continue

            prev[neighbor] = curr
            g_score[neighbor] = curr_g_score
            f_score[neighbor] = g_score[neighbor] + heuristic_cost(neighbor, end)

    # if we get to this point, it's not possible to reach the end destination
    return []

