import math
import numpy as np
import heapq

STEP_COST = 1.0
THRESHOLD_XY = 0.5
THRESHOLD_THETA = 30

# initialize 3D visited matrix
visited_matrix = np.zeros((1200, 500, 12), dtype=bool)

# 
def is_duplicate(x, y, theta):
    # coordinates to matrix indices
    x_idx = int(round(x / THRESHOLD_XY))
    y_idx = int(round(y / THRESHOLD_XY))
    theta_idx = int((theta % 360) // THRESHOLD_THETA)
    
    # edge case -  out of bounds
    if x_idx < 0 or x_idx >= 1200 or y_idx < 0 or y_idx >= 500:
        return True 

    if visited_matrix[x_idx][y_idx][theta_idx]:
        return True
    else:
        # mark visited
        visited_matrix[x_idx][y_idx][theta_idx] = True
        return False

# new state after taking a step
def move(x, y, theta, step_size, angle_change):
    new_theta = (theta + angle_change) % 360
    
    # degrees to radians
    theta_rad = math.radians(new_theta)
    
    # get new continuous coordinates
    new_x = x + step_size * math.cos(theta_rad)
    new_y = y + step_size * math.sin(theta_rad)
    
    # cap decimals
    return round(new_x, 2), round(new_y, 2), new_theta

# all steering actions
def action_minus_60(x, y, theta, L):
    return move(x, y, theta, L, -60)

def action_minus_30(x, y, theta, L):
    return move(x, y, theta, L, -30)

def action_0(x, y, theta, L):
    return move(x, y, theta, L, 0)

def action_plus_30(x, y, theta, L):
    return move(x, y, theta, L, 30)

def action_plus_60(x, y, theta, L):
    return move(x, y, theta, L, 60)

# helper
def get_neighbors(x, y, theta, L):
    return [
        action_minus_60(x, y, theta, L),
        action_minus_30(x, y, theta, L),
        action_0(x, y, theta, L),
        action_plus_30(x, y, theta, L),
        action_plus_60(x, y, theta, L)
    ]
    
# heuristic
def calculate_heuristic(x, y, target_x, target_y):
    return math.sqrt((x - target_x)**2 + (y - target_y)**2)

def is_goal_reached(x, y, target_x, target_y):
    return calculate_heuristic(x, y, target_x, target_y) <= 1.5

# continuous to discrete
def get_discrete_index(x, y, theta):
    x_idx = int(round(x / THRESHOLD_XY))
    y_idx = int(round(y / THRESHOLD_XY))
    theta_idx = int((theta % 360) // THRESHOLD_THETA)
    return (x_idx, y_idx, theta_idx)

# main method - backward a star
def backward_a_star(user_start, user_goal, L, map_img, is_valid_func):
    # get the initial and goal node from the user (but backward)
    Xi = user_goal
    Xg = user_start
    
    open_list = []
    visited_info = {} 
    
    # add initial node to OpenList
    heapq.heappush(open_list, (0, 0, Xi, Xi))
    start_idx = get_discrete_index(*Xi)
    visited_info[start_idx] = {'cost': 0, 'state': Xi, 'parent': None}
    
    # tracking explored nodes for animation
    explored_nodes = [] 
    
    # While OpenList is not empty and not at goal    
    while open_list:
        # pop off the openlist
        curr_f, curr_c2c, x, parent_state = heapq.heappop(open_list)
        cx, cy, ct = x
        x_idx = get_discrete_index(cx, cy, ct)
        
        # Add to closed set
        visited_matrix[x_idx[0]][x_idx[1]][x_idx[2]] = True
        
        # save
        explored_nodes.append((x, parent_state))
        
        # exit if goal reached
        if is_goal_reached(cx, cy, Xg[0], Xg[1]):
            return explored_nodes, visited_info, x
            
        # else
        else:
            # forall all neighbors
            neighbors = get_neighbors(cx, cy, ct, L)
            for neighbor in neighbors:
                # get all valid actions of neighbor
                nx, ny, nt = neighbor
                n_idx = get_discrete_index(nx, ny, nt)
                
                # check - make sure indices are in bounds
                if n_idx[0] < 0 or n_idx[0] >= 1200 or n_idx[1] < 0 or n_idx[1] >= 500:
                    continue
                    
                # check - make sure indice is valid (not visited and not an obstacle)
                if not visited_matrix[n_idx[0]][n_idx[1]][n_idx[2]] and is_valid_func(nx, ny):
                    
                    # if neighbor is not in openlist and cost to come is not set
                    if n_idx not in visited_info:
                        parent_neighbor = x
                        # cost to come, cost to go, cost of neighbor
                        c2c_neighbor = curr_c2c + STEP_COST
                        cost_to_go = calculate_heuristic(nx, ny, Xg[0], Xg[1])
                        cost_neighbor = c2c_neighbor + cost_to_go
                        
                        # put neighbor in openlist
                        visited_info[n_idx] = {'cost': c2c_neighbor, 'state': neighbor, 'parent': parent_neighbor}
                        heapq.heappush(open_list, (cost_neighbor, c2c_neighbor, neighbor, parent_neighbor))
                        
                    # else
                    else:
                        # if cost to come is greater than current cost, update cost + parent
                        if visited_info[n_idx]['cost'] > curr_c2c + STEP_COST:
                            parent_neighbor = x
                        # cost to come, cost to go, cost of neighbor
                            c2c_neighbor = curr_c2c + STEP_COST
                            cost_to_go = calculate_heuristic(nx, ny, Xg[0], Xg[1])
                            cost_neighbor = c2c_neighbor + cost_to_go
                            
                            # push updated costs to openlist
                            visited_info[n_idx] = {'cost': c2c_neighbor, 'state': neighbor, 'parent': parent_neighbor}
                            heapq.heappush(open_list, (cost_neighbor, c2c_neighbor, neighbor, parent_neighbor))

    # if no path found yet, return failure + message
    print("Search failed, no was path found.")
    return None, None, None