import math
import numpy as np
import heapq

THRESHOLD_XY = 5.0
THRESHOLD_THETA = 30
ACTION_TIME = 1

# turtlebot specs cm 
R = 3.8
L = 35.4

visited_matrix = np.zeros((81, 41, 13), dtype=bool)

# differential integration move
def move(x, y, theta_rad, ul_rpm, ur_rpm, map_img, is_valid_func):
    t = 0
    dt = 0.1
    
    # Convert rpm to rad/sec
    ul = ul_rpm * (2 * math.pi / 60)
    ur = ur_rpm * (2 * math.pi / 60)
    
    path_points = []
    cost = 0
    curr_x, curr_y, curr_theta = x, y, theta_rad
    
    while t < ACTION_TIME:
        t += dt
        dx = 0.5 * R * (ul + ur) * math.cos(curr_theta) * dt
        dy = 0.5 * R * (ul + ur) * math.sin(curr_theta) * dt
        dtheta = (R / L) * (ur - ul) * dt
        
        curr_x += dx
        curr_y += dy
        curr_theta += dtheta
        cost += math.sqrt(dx**2 + dy**2)
        
        curr_theta = curr_theta % (2 * math.pi)
        
        # Check collision along the generated curve
        if not is_valid_func(curr_x, curr_y, map_img):
            return None
            
        path_points.append((curr_x, curr_y, curr_theta))
        
    return (curr_x, curr_y, curr_theta), path_points, cost

def get_neighbors(x, y, theta_rad, rpm1, rpm2, map_img, is_valid_func):
    actions = [
        (0, rpm1), (rpm1, 0), (rpm1, rpm1), 
        (0, rpm2), (rpm2, 0), (rpm2, rpm2), 
        (rpm1, rpm2), (rpm2, rpm1)
    ]
    
    valid_neighbors = []
    for ul, ur in actions:
        result = move(x, y, theta_rad, ul, ur, map_img, is_valid_func)
        if result is not None:
            final_state, path, cost = result
            valid_neighbors.append((final_state, path, cost, (ul, ur)))
    return valid_neighbors

def calculate_heuristic(x, y, target_x, target_y):
    return math.sqrt((x - target_x)**2 + (y - target_y)**2)

def is_goal_reached(x, y, target_x, target_y, threshold=10.0): # 10cm radius
    return calculate_heuristic(x, y, target_x, target_y) <= threshold

def get_discrete_index(x, y, theta_rad):
    x_idx = int(x // THRESHOLD_XY)
    y_idx = int(y // THRESHOLD_XY)
    theta_deg = math.degrees(theta_rad)
    theta_idx = int((theta_deg % 360) // THRESHOLD_THETA)
    return (x_idx, y_idx, theta_idx)

def forward_a_star(user_start, user_goal, rpm1, rpm2, map_img, is_valid_func):
    open_list = []
    visited_info = {} 
    
    # Start node logic
    sx, sy, stheta_deg = user_start
    stheta_rad = math.radians(stheta_deg)
    start_state = (sx, sy, stheta_rad)
    
    heapq.heappush(open_list, (0, 0, start_state, start_state, [], (0,0)))
    start_idx = get_discrete_index(*start_state)
    visited_info[start_idx] = {'cost': 0, 'state': start_state, 'parent': None, 'path': [], 'action': (0,0)}
    
    explored_nodes = [] 
    
    print("Searching for path")
    
    while open_list:
        curr_f, curr_c2c, curr_state, parent_state, path_from_parent, action = heapq.heappop(open_list)
        cx, cy, ct = curr_state
        x_idx = get_discrete_index(cx, cy, ct)
        
        visited_matrix[x_idx[0]][x_idx[1]][x_idx[2]] = True
        explored_nodes.append((curr_state, path_from_parent))
        
        if is_goal_reached(cx, cy, user_goal[0], user_goal[1]):
            return explored_nodes, visited_info, curr_state
            
        neighbors = get_neighbors(cx, cy, ct, rpm1, rpm2, map_img, is_valid_func)
        for neighbor_state, path, step_cost, (ul, ur) in neighbors:
            nx, ny, nt = neighbor_state
            n_idx = get_discrete_index(nx, ny, nt)
            
            #
            if n_idx[0] < 0 or n_idx[0] >= 81 or n_idx[1] < 0 or n_idx[1] >= 41 or n_idx[2] < 0 or n_idx[2] >= 13:
                continue
                
            if not visited_matrix[n_idx[0]][n_idx[1]][n_idx[2]]:
                turn_penalty = abs(ur - ul) * 0.03
                c2c_neighbor = curr_c2c + step_cost + turn_penalty

                cost_to_go = calculate_heuristic(nx, ny, user_goal[0], user_goal[1])
                cost_neighbor = c2c_neighbor + (1.0 * cost_to_go)
                
                if n_idx not in visited_info or visited_info[n_idx]['cost'] > c2c_neighbor:
                    visited_info[n_idx] = {
                        'cost': c2c_neighbor, 
                        'state': neighbor_state, 
                        'parent': curr_state,
                        'path': path,
                        'action': (ul, ur)
                    }
                    heapq.heappush(open_list, (cost_neighbor, c2c_neighbor, neighbor_state, curr_state, path, (ul, ur)))

    print("Search failed, no path was found.")
    return None, None, None

def backtrack(visited_info, final_state, start_state):
    full_path_curves = []
    actions_taken = []
    curr_state = final_state
    
    while curr_state is not None:
        curr_idx = get_discrete_index(*curr_state)
        info = visited_info.get(curr_idx)
        
        if info['parent'] is None:
            break
            
        full_path_curves.append(info['path'])
        actions_taken.append(info['action'])
        curr_state = info['parent']
        
    full_path_curves.reverse()
    actions_taken.reverse()
    return full_path_curves, actions_taken