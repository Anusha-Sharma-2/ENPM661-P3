import os
import cv2
import map
from a_star_anusha_sharma import *

ROBOT_RADIUS = 10.5  # cm
DEBUG_WAYPOINT_LIMIT = 100

if __name__ == '__main__':
    # get user inputs
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

            # validate input
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

        # Save actions like before
        actions_path = os.path.join(base_dir, "actions.txt")
        with open(actions_path, "w") as f:
            for ul, ur in actions:
                f.write(f"{ul},{ur}\n")

        print(f"Exported actions to {actions_path}")

        # # Save waypoints for odom-based P controller
        # waypoints_path = os.path.join(base_dir, "waypoints.txt")
        # Save waypoints for odom-based P controller
        
        # part02_dir = os.path.abspath(os.path.join(base_dir, "..", "Part02/turtlebot_planner/turtlebot_planner"))
        # if not os.path.exists(part02_dir):
        #     os.makedirs(part02_dir)            
        # waypoints_path = os.path.join(part02_dir, "waypoints.txt")

        # base_dir = os.path.dirname(os.path.abspath(__file__))
        # # This goes up one level to the /root/proj3_ws/ENPM661-P3/ folder
        # project_root = os.path.abspath(os.path.join(base_dir, ".."))
        # waypoints_path = os.path.join(project_root, "waypoints.txt")

        waypoints_path = os.path.expanduser("~/waypoints.txt")

        # Planner map: 400 x 200 cm
        # Gazebo arena: about 8 x 4 m
        GAZEBO_SCALE = 2.0

        # From /odom, turtlebot starts around x=0.5, y=0.0
        GAZEBO_START_X = 0.50
        GAZEBO_START_Y = 0.0

        PLANNER_START_X = xs
        PLANNER_START_Y = ys

        planner_waypoints = []
        gazebo_waypoints = []

        with open(waypoints_path, "w") as f:
            for curve in optimal_path:
                if len(curve) == 0:
                    continue

                # Save dense waypoints along each A* curve.
                for i, point in enumerate(curve):
                    if i % 12 != 0 and i != len(curve) - 1:
                        continue

                    x_cm, y_cm, theta_rad = point

                    # Save planner-space waypoint for debug drawing
                    planner_waypoints.append((x_cm, y_cm))

                    # Convert planner-relative cm motion to Gazebo-relative meters.
                    x_m = GAZEBO_START_X + ((x_cm - PLANNER_START_X) / 100.0) * GAZEBO_SCALE
                    y_m = GAZEBO_START_Y + ((y_cm - PLANNER_START_Y) / 100.0) * GAZEBO_SCALE

                    gazebo_waypoints.append((x_m, y_m))
                    f.write(f"{x_m},{y_m}\n")

        print(f"Exported waypoints to {waypoints_path}")
        print(f"Generated {len(gazebo_waypoints)} Gazebo waypoints.")

        # Save a debug image showing only the first few waypoints/path segments
        debug_first = map.generate_map(ROBOT_RADIUS, clearance)

        # draw start and goal
        cv2.circle(
            debug_first,
            (int(xs), map.HEIGHT - 1 - int(ys)),
            5,
            (0, 255, 0),
            -1
        )

        cv2.circle(
            debug_first,
            (int(xg), map.HEIGHT - 1 - int(yg)),
            5,
            (255, 0, 255),
            -1
        )

        limited_waypoints = planner_waypoints[:DEBUG_WAYPOINT_LIMIT]

        for i, (wx, wy) in enumerate(limited_waypoints):
            px = int(wx)
            py = map.HEIGHT - 1 - int(wy)

            # first waypoint is yellowish rest are blue
            color = (0, 255, 255) if i == 0 else map.BLUE
            cv2.circle(debug_first, (px, py), 3, color, -1)

            if i > 0:
                prev_x, prev_y = limited_waypoints[i - 1]
                pt1 = (int(prev_x), map.HEIGHT - 1 - int(prev_y))
                pt2 = (int(wx), map.HEIGHT - 1 - int(wy))
                cv2.line(debug_first, pt1, pt2, map.BLUE, 2)

        if len(limited_waypoints) > 0:
            last_x, last_y = limited_waypoints[-1]
            cv2.circle(
                debug_first,
                (int(last_x), map.HEIGHT - 1 - int(last_y)),
                7,
                (0, 128, 255),
                -1
            )

        debug_first_path = os.path.join(base_dir, f"debug_first_{DEBUG_WAYPOINT_LIMIT}_waypoints.png")
        cv2.imwrite(debug_first_path, debug_first)
        print(f"Saved first {DEBUG_WAYPOINT_LIMIT} waypoint debug image to {debug_first_path}")

        # save the first few waypoint numbers to a text file for debugging
        debug_txt_path = os.path.join(base_dir, f"debug_first_{DEBUG_WAYPOINT_LIMIT}_waypoints.txt")
        with open(debug_txt_path, "w") as f:
            f.write("idx,planner_x_cm,planner_y_cm,gazebo_x_m,gazebo_y_m\n")
            for i in range(min(DEBUG_WAYPOINT_LIMIT, len(planner_waypoints))):
                px, py = planner_waypoints[i]
                gx, gy = gazebo_waypoints[i]
                f.write(f"{i},{px},{py},{gx},{gy}\n")

        print(f"Saved waypoint debug text to {debug_txt_path}")

    else:
        print("Exiting without animation because no path was found.")
        exit()

    print("Starting animation generation:")

    # video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter("animation.mp4", fourcc, 60.0, (map.WIDTH, map.HEIGHT))

    # animate explored nodes
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

    # animate optimal path
    if optimal_path:
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