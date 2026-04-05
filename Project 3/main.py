import map
from a_star_anusha_sharma import *
import cv2

if __name__ == '__main__':
    # generate map
    workspace = map.generate_map()
    
    # get user inputs 
    while True:
        try:
            print("\nEnter start coordinates:")
            xs = float(input("X coordinate (0-600): "))
            ys = float(input("Y coordinate (0-250): "))
            ts = float(input("Theta (degrees, multiple of 30): "))
            
            print("\nEnter goal coordinates:")
            xg = float(input("X coordinate (0-600): "))
            yg = float(input("Y coordinate (0-250): "))
            tg = float(input("Theta (degrees, multiple of 30): "))
            
            L = float(input("\nEnter Step Size L (1 to 10): "))
            
            # validate input
            if not map.is_valid_node(xs, ys, workspace):
                print("ERROR: Start node is in an obstacle or out of bounds! Try again.")
                continue
            if not map.is_valid_node(xg, yg, workspace):
                print("ERROR: Goal node is in an obstacle or out of bounds! Try again.")
                continue
            if L < 1 or L > 10:
                print("ERROR: Step size must be between 1 and 10! Try again.")
                continue
                
            break
            
        except ValueError:
            print("Invalid input. Please enter numbers only.")

    user_start = (xs, ys, ts)
    user_goal = (xg, yg, tg)
    
    # run algorithm
    explored, visited, final_node = backward_a_star(user_start, user_goal, L, workspace, lambda x, y: map.is_valid_node(x, y, workspace))
    
    # backtrack
    if final_node is not None:
        optimal_path = backtrack(visited, final_node, user_goal)
        print(f"Path generated with {len(optimal_path)} steps!")
        
    print("Starting animation generation:")
        
    # video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('animation.mp4', fourcc, 60.0, (map.WIDTH, map.HEIGHT))

    # animating node exploration
    for i, (curr_state, parent_state) in enumerate(explored):
        if curr_state == parent_state:
            continue
                
        cx, cy, _ = curr_state
        px, py, _ = parent_state
            
        # continuous to discrete (for drawing pixels sake)
        pt1 = (int(px), map.HEIGHT - 1 - int(py))
        pt2 = (int(cx), map.HEIGHT - 1 - int(cy))
            
        # robot's movement is red line
        cv2.line(workspace, pt1, pt2, map.RED, 1)
            
        # for every 50th frame, render frame
        if i % 50 == 0:
            out.write(workspace)

    # animate path
    if optimal_path:
        # loop through the path array, connecting i-1 to i
        for i in range(1, len(optimal_path)):
            px, py, _ = optimal_path[i-1]
            cx, cy, _ = optimal_path[i]
            pt1 = (int(px), map.HEIGHT - 1 - int(py))
            pt2 = (int(cx), map.HEIGHT - 1 - int(cy))
                
            # final path is blue line
            cv2.line(workspace, pt1, pt2, map.BLUE, 2)
            out.write(workspace)

    # wait at end to show results
    for _ in range(200):
        out.write(workspace)

    out.release()
    
    print("Animation saved as animation.mp4!")
