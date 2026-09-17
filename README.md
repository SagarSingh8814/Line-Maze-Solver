# Line-Maze-Solver
xpecto-lms — Line Maze Solver
An autonomous line-following robot that explores an unknown maze, builds an internal wall map, then computes and runs the shortest path using flood-fill.
Hardware
ESP32-DEVKITC
L293D dual motor driver
5x IR line sensors
Push button (start/mode select)
Custom PCB (schematic included as Schematic_xpecto-lms.pdf / image)
How it works
Line following — PID control across the 5-sensor IR array keeps the robot centered on the line.
Exploration phase — at each junction, the robot picks a direction (left-hand-rule fallback), logs which walls it finds, and updates its position on an internal grid.
Flood-fill — once the goal is reached, a BFS-based flood-fill recomputes the shortest distance from every visited cell to the goal, respecting discovered walls.
Optimized run — the robot re-runs the maze, at each cell greedily moving to the neighbor with the lowest flood-fill distance, which guarantees the shortest path.
Files
maze_solver.ino — full source: PID line following + flood-fill maze solving on ESP32
Schematic_xpecto-lms.png — circuit schematic (ESP32, L293D, sensor headers)
Notes
Junction-detection thresholds and turn timing are chassis-specific and were tuned empirically on the physical robot. Pin assignments in the code follow the schematic; double check GPIO numbers against your board silkscreen before flashing.
