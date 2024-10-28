# MAPS Multi Agent Pathfinding Simulator
The Multi-Agent Pathfinding Simulator (MAPS) calculates and simulates coordinated movement of multiple agents within a 2D grid-based environment. Using the A* algorithm for pathfinding, it incorporates spatio-temporal reservations to ensure collision-free navigation. By reserving both grid nodes and movement timings, agents avoid conflicts like overlapping positions and shared paths.

It implements a priority-based approach, where each agent is assigned a priority level that influences conflict resolution. Higher-priority agents are able to move with minimal delay, while lower-priority agents adapt their paths around the routes of higher-priority agents to avoid collisions. This structured approach enables efficient and orderly movement across the grid.

The simulator includes a real-time graphical interface that visualizes agent paths, priorities and metrics, providing valuable insights into pathfinding behavior and priority-based coordination.
<br />

![image](https://github.com/user-attachments/assets/2bdb6224-3770-4245-9044-6a6bbc5ce610)

## Simulation Metrics
- ```Flowtime``` : The total sum of travel times for all agents within the system.
- ```MakeSpan``` : The travel time of the final agent to reach its destination.
- ```Conflicts Resolved``` : Number of conflicts resolved by the system that would otherwise be present when using the A* algorithm with no collision avoidance.

## Configuration and Use
By default the pathfinding system has an environment resolution of ```100x100``` and ```20``` active Agents.

Environment data is stored in the ```map.json``` file. This can be modified to set up custom environments.

You can interact with the simulator using the following keyboard inputs:

- ```1``` Enable Grid Overlay
- ```2``` Disable Grid Overlay
- ```3``` Begin/Resume Simulation
- ```4``` Pause Simulation
- ```5``` Reset Simulation (New agent and end point Configuration)

https://github.com/user-attachments/assets/d1e08001-8715-4860-8e86-cdd663827a27

