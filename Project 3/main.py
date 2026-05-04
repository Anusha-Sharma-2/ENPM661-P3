import os
import cv2
import map
from a_star_anusha_sharma import *

ROBOT_RADIUS = 10.5

if __name__ == '__main__':
    while True:
        try:
            print("\nEnter start coordinates:")
            xs = float(input(f"X coordinate (0-{map.WIDTH}): "))
            ys = float(input(f"Y coordinate (0-{map.HEIGHT}): "))
            ts = float(input("Theta in degrees: "))

            print("\nEnter goal coordinates:")
            xg = float(input(f"X coordinate (0-{map.WIDTH}): "))
            yg = float(input(f"Y coordinate (0-{map.HEIGHT}): "))

            rpm1 = float(input("\nEnter RPM 1: "))
            rpm2 = float(input("Enter RPM 2: "))
            clearance = float(input("Enter Clearance in cm: "))

            workspace = map.generate_map(ROBOT_RADIUS, clearance)
            cv2.imwrite("debug_map.png", workspace)

            test_points = [
                (0, 100),
                (25, 100),
                (50, 100),
                (60, 100),
                (75, 100),
                (100, 100),
                (350, 100)
            ]

            print("\nValidity check:")
            for tx, ty in test_points:
                print(tx, ty, map.is_valid_node(tx, ty, workspace))
            
            debug = workspace.copy()

            for tx, ty in test_points:
                color = (0, 255, 0) if map.is_valid_node(tx, ty, workspace) else (0, 0, 255)
                cv2.circle(debug, (int(tx), map.HEIGHT - 1 - int(ty)), 4, color, -1)

            cv2.imwrite("debug_points.png", debug)

            if not map.is_valid_node(xs, ys, workspace):
                print("ERROR: Start node is in an obstacle or out of bounds. Try again.")
                continue

            if not map.is_valid_node(xg, yg, workspace):
                print("ERROR: Goal node is in an obstacle or out of bounds. Try again.")
                continue

            break

        except ValueError:
            print("Invalid input. Please enter numbers only.")

    user_start = (xs, ys, ts)
    user_goal = (xg, yg)

    explored, visited, final_node = forward_a_star(
        user_start,
        user_goal,
        rpm1,
        rpm2,
        workspace,
        lambda x, y, img: map.is_valid_node(x, y, img)
    )

    optimal_path = []
    actions = []

    if final_node is not None:
        optimal_path, actions = backtrack(visited, final_node, user_start)
        print(f"Path generated with {len(optimal_path)} steps.")

        base_dir = os.path.dirname(os.path.abspath(__file__))
        actions_path = os.path.join(base_dir, "actions.txt")

        with open(actions_path, "w") as f:
            for ul, ur in actions:
                f.write(f"{ul},{ur}\n")

        print(f"Exported actions to {actions_path}")

    else:
        print("Exiting without animation because no path was found.")
        exit()

    print("Starting animation generation.")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter("animation.mp4", fourcc, 60.0, (map.WIDTH, map.HEIGHT))

    for i, (curr_state, path_curve) in enumerate(explored):
        if len(path_curve) < 2:
            continue

        for j in range(1, len(path_curve)):
            px, py, _ = path_curve[j - 1]
            cx, cy, _ = path_curve[j]

            pt1 = (int(px), map.HEIGHT - 1 - int(py))
            pt2 = (int(cx), map.HEIGHT - 1 - int(cy))

            cv2.line(workspace, pt1, pt2, map.RED, 1)

        if i % 50 == 0:
            out.write(workspace)

    for curve in optimal_path:
        for i in range(1, len(curve)):
            px, py, _ = curve[i - 1]
            cx, cy, _ = curve[i]

            pt1 = (int(px), map.HEIGHT - 1 - int(py))
            pt2 = (int(cx), map.HEIGHT - 1 - int(cy))

            cv2.line(workspace, pt1, pt2, map.BLUE, 2)

        out.write(workspace)

    for _ in range(200):
        out.write(workspace)

    out.release()
    print("Animation saved as animation.mp4.")