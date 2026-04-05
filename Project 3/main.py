import map
from a_star import *

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
        
