

# best path planner for the o'rizzler: 
Dynamic path planning is a critical aspect of robotics and autonomous navigation, where the environment is subject to changes, requiring real-time updates to the planned path. The **D* (Dynamic A*) algorithm** is an extension of the A* algorithm, designed for dynamic environments where obstacles may change over time.

## Overview of D* Algorithm
D* (Dynamic A*) is an optimal and efficient algorithm used for re-planning paths in changing environments. Unlike A*, which plans a static path, D* updates the path dynamically as new information about the environment is received.

### Key Features of D*:
- **Dynamic Re-planning:** Efficiently updates the path when obstacles appear or disappear.
- **Optimal Pathfinding:** Finds the shortest or least-cost path from the start position to the goal.
- **Efficient Computation:** Reduces computational overhead compared to re-running A* from scratch.
- **Applicable in Robotics:** Frequently used in autonomous robots, UAVs, and mobile robots.

## Steps Involved in D* Algorithm
1. **Initialization:**
   - Define the grid/map with traversable and non-traversable areas.
   - Set start and goal positions.
   - Initialize cost values and priority queue.

2. **Path Calculation (Similar to A*):**
   - Use a heuristic (e.g., Euclidean or Manhattan distance) to estimate the cost.
   - Expand the least-cost node and update neighbors.
   - Store parent nodes for path reconstruction.

3. **Dynamic Obstacle Handling:**
   - If an obstacle appears, re-evaluate the path.
   - Update affected nodes and propagate cost changes backward to find an alternate route.

4. **Re-planning and Execution:**
   - The agent follows the computed path.
   - If new obstacles are detected, the algorithm re-plans the path dynamically.

## Comparison with Other Path Planning Algorithms
| Algorithm | Environment | Re-planning | Computational Efficiency |
|-----------|------------|-------------|--------------------------|
| A*        | Static     | No          | Moderate                 |
| D*        | Dynamic    | Yes         | High                     |
| D* Lite   | Dynamic    | Yes         | More efficient than D*   |


## Conclusion
for o'rizzler to find the perfect way this method is better method since it has encountered all the other obstacles and just gave the rizzler the perfect path to go to the destination 

### the other methods tried and would have been better:

### RRT method:
the rapidly exploring random tree method is an algorithm used for path planning by using robotics and autonomous systems. It is a particulary useful for high dimensional spaces and environments with obstacles.

 - initialise the tree node
 - randomly sample the point in the environment
 - find the nearest node in the tree to the sampled point.
 - steer towards the sampled point avoiding all the obstacles
 - makes the path and extracts it to attain the goal.
