import map
from a_star_anusha_sharma import *
import cv2

if __name__ == '__main__':
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
            
            rpm1 = float(input("\nEnter RPM 1: "))
            rpm2 = float(input("Enter RPM 2: "))
            clearance = float(input("Enter Clearance (cm): "))
            
            # Generate map after we get clearance, assuming standard 22cm TurtleBot radius
            workspace = map.generate_map(22.0, clearance)
            
            # validate input
            if not map.is_valid_node(xs, ys, workspace):
                print("ERROR: Start node is in an obstacle or out of bounds! Try again.")
                continue
            if not map.is_valid_node(xg, yg, workspace):
                print("ERROR: Goal node is in an obstacle or out of bounds! Try again.")
                continue
                
            break
            
        except ValueError:
            print("Invalid input. Please enter numbers only.")

    user_start = (xs, ys, ts)
    user_goal = (xg, yg, tg)
    
    # run algorithm (switched to forward_a_star and passed rpms)
    explored, visited, final_node = forward_a_star(user_start, user_goal, rpm1, rpm2, workspace, lambda x, y, img: map.is_valid_node(x, y, img))
    
    # backtrack
    if final_node is not None:
        # Backtrack now returns the curves and the actions
        optimal_path, actions = backtrack(visited, final_node, user_start)
        print(f"Path generated with {len(optimal_path)} steps!")
        
        # Save actions for the ROS 2 open-loop controller
        with open("actions.txt", "w") as f:
            for ul, ur in actions:
                f.write(f"{ul},{ur}\n")
        print("Exported actions.txt for Gazebo Simulation.")
        
    print("Starting animation generation:")
        
    # video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('animation.mp4', fourcc, 60.0, (map.WIDTH, map.HEIGHT))

    # animating node exploration (modified to draw curves instead of straight lines)
    for i, (curr_state, path_curve) in enumerate(explored):
        if len(path_curve) < 2:
            continue
                
        for j in range(1, len(path_curve)):
            px, py, _ = path_curve[j-1]
            cx, cy, _ = path_curve[j]
            
            # continuous to discrete (for drawing pixels sake)
            pt1 = (int(px), map.HEIGHT - 1 - int(py))
            pt2 = (int(cx), map.HEIGHT - 1 - int(cy))
                
            # robot's movement is red curve
            cv2.line(workspace, pt1, pt2, map.RED, 1)
            
        # for every 50th frame, render frame
        if i % 50 == 0:
            out.write(workspace)

    # animate path (modified to draw the optimal path curves)
    if optimal_path:
        for curve in optimal_path:
            for i in range(1, len(curve)):
                px, py, _ = curve[i-1]
                cx, cy, _ = curve[i]
                pt1 = (int(px), map.HEIGHT - 1 - int(py))
                pt2 = (int(cx), map.HEIGHT - 1 - int(cy))
                    
                # final path is blue curve
                cv2.line(workspace, pt1, pt2, map.BLUE, 2)
            out.write(workspace)

    # wait at end to show results
    for _ in range(200):
        out.write(workspace)

    out.release()
    
    print("Animation saved as animation.mp4!")