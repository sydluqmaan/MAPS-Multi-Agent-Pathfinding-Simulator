import numpy as np
import matplotlib.pyplot as plt
import config as cfg
import random
import math
import json
import matplotlib.colors as mcolors
import heapq
import copy
import time
import os
import sys

class Environment:

    class StoredComponent:

        def __init__ (self, typeIn, origin_x, origin_y, size_x, size_y):
            self.type = typeIn
            self.x = origin_x
            self.y = origin_y
            self.width = size_x
            self.height = size_y
        
        def getType(self):
            return self.type
        
        def getX(self):
            return self.x
        
        def getY(self):
            return self.y
        
        def getWidth(self):
            return self.width
        
        def getHeight(self):
            return self.height
        
    def __init__(self, gridResolutionIn):

        if getattr(sys, 'frozen', False):
            current_dir = os.path.dirname(sys.executable)
        else:
            current_dir = os.path.dirname(__file__)

        filePath = os.path.join(current_dir, 'map.json')

        self.scene_components = None

        # Parse environment data from json
        def obj_from_json(data):
            return self.StoredComponent(data['type'], data['attributes']['x'], data['attributes']['y'], data['attributes']['width'], data['attributes']['height'])

        # Instantiate environment objects
        with open(filePath, 'r') as file:
            data = json.load(file)
            self.scene_components = [obj_from_json(item) for item in data]

        # Initialize empty environment grid
        self.grid = np.zeros(gridResolutionIn, dtype=int)
        gridUnitLength = round(1000/gridResolutionIn[0]) #25
        self.grid_res = gridResolutionIn
            
        for obj in self.scene_components:
            x, y, width, height = obj.getX(), obj.getY(), obj.getWidth(), obj.getHeight()
    
            start_col = max(x // gridUnitLength, 0)
            end_col = min((x + width - 1) // gridUnitLength, gridResolutionIn[0])
            start_row = max(y // gridUnitLength, 0)
            end_row = min((y + height - 1) // gridUnitLength, gridResolutionIn[0])
            
            # Mark the grid cells
            for i in range(start_row, end_row + 1):
                for j in range(start_col, end_col + 1):
                    self.grid[i, j] = 1
        
        self.grid = np.flipud(self.grid)

    def getComponents(self):
        return self.scene_components

    def show_grid(self, route):
        offsetX = 0 
        offsetY = 0 
        extent = [offsetX, self.grid.shape[1] + offsetX, offsetY, self.grid.shape[0] + offsetY]

        cmap = mcolors.ListedColormap(['white', 'black', 'red', 'green', 'orange'])

        tiles = self.grid.copy()

        if route:
            for tile in route:
                tiles[tile[0]][tile[1]] = 4

        plt.imshow(tiles, cmap=cmap, extent=extent)

        plt.xticks(range(self.grid.shape[1]))
        plt.yticks(range(self.grid.shape[0]))

        plt.grid(which='both', color='gray', linestyle='-', linewidth=0.5)

        plt.xlabel('X axis')
        plt.ylabel('Y axis')

        plt.gca().invert_yaxis()

        plt.show()

    def isOccupied(self, coordinatesIn):
        return self.grid[coordinatesIn[0]][coordinatesIn[1]]
        
    def get_grid_res(self):
        return self.grid_res

class EnvironmentAgent:
    
    def __init__(self, idIn, colorIn, locationIn, goalIn):
        self.id = idIn
        self.color = colorIn
        self.location = locationIn
        self.path = None
        self.goal = goalIn
        self.distance = 0
        self.priority = 0

    def getID(self):
        return self.id
    
    def getColor(self):
        return self.color

    def get_location(self):
        return self.location
    
    def get_path(self):
        return self.path
    
    def setPriority(self, priorityIn):
        self.priority = priorityIn

    def set_location(self, locIn):
        self.location = locIn

    def set_path(self, pathIn):
        self.path = pathIn

    def get_goal(self):
        return self.goal
    
    def __lt__(self, other):
        return self.distance > other.distance

class Director:

    def __init__(self, envIn, simulationIn):
        self.agents = []
        self.color_index = 0
        self.env = envIn
        self.simulation = simulationIn
        self.simulation_speed = 5
        self.time_stamp = None
        self.priorityIndex = 0

    def resetDirector(self):
        self.agents = []
        self.color_index = 0

    def getNextColor(self):
        self.color_index = self.color_index + 1 
        if self.color_index == len(cfg.color_list):
            self.color_index = 0 
        return cfg.color_list[self.color_index]
    
    def generateAgentLocation(self):
        coordinates = None
        x = None

        valid = False
        while valid == False:
            coordinates = (random.randint(0, self.env.get_grid_res()[0]-1), random.randint(0, self.env.get_grid_res()[0]-1))
            
            if self.env.isOccupied(coordinates)==0:

                is_overlapping = False
                for agent in self.agents:
                    x_distance = (coordinates[0]-agent.get_location()[0])**2
                    y_distance = (coordinates[1]-agent.get_location()[1])**2

                    dist = math.sqrt(x_distance + y_distance)

                    if dist <= 5:
                        is_overlapping = True
                
                valid = not is_overlapping
        
        return coordinates
                
    def generateGoalLocation(self, ownerLoc):
        coordinates = None
        x = None

        valid = False
        while valid == False:
            coordinates = (random.randint(0, self.env.get_grid_res()[0]-1), random.randint(0, self.env.get_grid_res()[0]-1))
            owner_x_distance = (coordinates[0]-ownerLoc[0])**2
            owner_y_distance = (coordinates[1]-ownerLoc[1])**2
            owner_dist = math.sqrt(owner_x_distance + owner_y_distance)
            
            if (self.env.isOccupied(coordinates)==0) and (owner_dist >= 5):
                x = self.env.isOccupied(coordinates)

                is_overlapping = False
                for agent in self.agents:
                    x_distance = (coordinates[0]-agent.get_location()[0])**2
                    y_distance = (coordinates[1]-agent.get_location()[1])**2

                    goal_x_distance = (coordinates[0]-agent.get_goal()[0])**2
                    goal_y_distance = (coordinates[1]-agent.get_goal()[1])**2

                    dist = math.sqrt(x_distance + y_distance)
                    goal_dist = math.sqrt(goal_x_distance + goal_y_distance)

                    if (dist <= 2) and (goal_dist <= 2):
                        is_overlapping = True
                
                valid = not is_overlapping
        
        return coordinates

    def initializeAgents(self, number):
        
        for idIn in range(0,number):
            agentLoc = self.generateAgentLocation()
            goalLoc = self.generateGoalLocation(agentLoc)
            agent = EnvironmentAgent(str(idIn), self.getNextColor(), agentLoc, goalLoc)
            self.agents.append(agent)
            self.env.grid[agentLoc[0]][agentLoc[1]] = 2
            self.env.grid[goalLoc[0]][goalLoc[1]] = 3

        self.simulation.init_agents(self.agents)

    def initializePaths(self):
        agentPaths = {}

        for agent in self.agents:
            route = self.calculatePathAStar(agent, self.env)
            agent.set_path(route)
        
        for agent in self.agents:
            agentPaths[agent] = agent.path
        conflicts1 = self.detectConflicts(agentPaths)
        
        # print("MAPF w/o collision avoidance | Conflicts: ")

        # for c in conflicts1:
        #     print("Agent A: " + str(c[0].getID()) + 
        #         " | Agent B: " + str(c[1].getID()) + 
        #         " | Coordinates: " + str(c[2]) + 
        #         " | Time: " + str(c[3]) + 
        #         " | Conflict Type: " + c[4])
        
        # print("")
        # print("-------------------------------------------")
        # print("")
        # print("PMAPF | Conflicts:")

        PMAPFSolver = PrioritizedMAPF(self, self.env)
        PMAPFSolver.begin(self.agents)

        self.simulation.init_paths()

        for agent in self.agents:
            agentPaths[agent] = agent.path
        conflicts2 = self.detectConflicts(agentPaths)

        # for c in conflicts2:
        #     print("Agent A: " + str(c[0].getID()) + 
        #         " | Agent B: " + str(c[1].getID()) + 
        #         " | Coordinates: " + str(c[2]) + 
        #         " | Time: " + str(c[3]) + 
        #         " | Conflict Type: " + c[4])
            
        return (conflicts1,conflicts2)

    class AStarNode:
        coordinates = None
        parent = None
        global_score = None
        local_score = None
        neighbours = []
        visited = False

        def __init__(self, loc):
            self.coordinates = loc

        def __lt__(self, other):
            return self.coordinates < other.coordinates

        def set_visited(self, state):
            self.visited = state
        
        def set_parent(self, parent):
            self.parent = parent

        def set_global_score(self, score):
            self.global_score = score

        def set_local_score(self, score):
            self.local_score = score

        def get_coordinates(self):
            return self.coordinates
        
        def append_neighbour(self, node):
            self.neighbours.append(node)

        def get_neighbours(self):
            return self.neighbours
        
        def get_visited(self):
            return self.visited
        
        def get_global_score(self):
            return self.global_score

        def get_local_score(self):
            return self.local_score
        
        def get_parent(self):
            return self.parent

    def euclideanDistance(self, p1, p2):
        x_distance = (p1[0]-p2[0])**2
        y_distance = (p1[1]-p2[1])**2

        return math.sqrt(x_distance + y_distance)

    def calculatePathAStar(self, agent, env):
        queue = []
        scene_resolution = env.grid_res
        scene = [[None for _ in range(scene_resolution[1])] for _ in range(scene_resolution[0])]
        start = agent.get_location()
        goal = agent.get_goal()

        starting_node = self.AStarNode(start)
        starting_node.set_local_score(0)
        starting_node.set_global_score(self.euclideanDistance(start, goal))
        scene[start[0]][start[1]] = starting_node

        goal_node = self.AStarNode(goal)
        scene[goal[0]][goal[1]] = goal_node
        
        heapq.heappush(queue, (starting_node.get_global_score(), starting_node))

        def check_node(check_node, current_node, goal_loc):
            new_local_score = current_node.get_local_score() + 1
            if check_node.get_local_score() is None or new_local_score < check_node.get_local_score():
                check_node.set_parent(current_node)
                check_node.set_local_score(new_local_score)
                check_node.set_global_score(self.euclideanDistance(check_node.get_coordinates(), goal_loc) + new_local_score)

        while queue:
            _, current_node = heapq.heappop(queue)
            x, y = current_node.get_coordinates()

            if current_node == goal_node:
                break  # Terminate when the goal is reached

            if current_node.get_visited():
                continue

            # Find neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # LEFT, RIGHT, TOP, BOTTOM
                nx, ny = x + dx, y + dy
                if 0 <= nx < scene_resolution[0] and 0 <= ny < scene_resolution[1] and env.isOccupied((nx, ny)) != 1:
                    neighbor_node = scene[nx][ny]
                    if neighbor_node is None:
                        neighbor_node = self.AStarNode((nx, ny))
                        scene[nx][ny] = neighbor_node
                    if not neighbor_node.get_visited():
                        check_node(neighbor_node, current_node, goal)
                        heapq.heappush(queue, (neighbor_node.get_global_score(), neighbor_node))

            current_node.set_visited(True)

        # Trace back the path
        path = []
        cursor = goal_node
        while cursor:
            path.append(cursor.get_coordinates())
            cursor = cursor.get_parent()

        path.reverse()

        return path

    def detectConflicts(self, agentPaths):

        motionConflictIdentifier = 0
        restingConflictIdentifier = 1

        agents = list(agentPaths.keys())
        paths = list(agentPaths.values())
        conflicts = []
        resting_positions = {}

        # Get goal locations
        for i, path in enumerate(paths):
            if path:
                goal_position = path[-1]
                resting_positions[i] = goal_position

        for i in range(len(paths)):
            for j in range(i + 1, len(paths)):
                path_a = paths[i]
                path_b = paths[j]
                # Check for vertex and edge conflicts
                for t in range(min(len(path_a), len(path_b))):
                    if path_a[t] == path_b[t]:  # Vertex conflict
                        conflicts.append((agents[i], agents[j], path_a[t], t, "Vertex Conflict"))

                    if (t < len(path_a) - 1 and t < len(path_b) - 1 and
                        path_a[t] == path_b[t + 1] and path_b[t] == path_a[t + 1]):  # Edge conflict
                        conflicts.append((agents[i], agents[j], path_a[t], t, "Edge Conflict"))

                # Check for conflicts with resting positions
                if i in resting_positions and t < len(path_b):
                    if path_b[t] == resting_positions[i]:  # Path B over resting position of A
                        conflicts.append((agents[i], agents[j], path_a[t], t, "Goal Overlap Conflict - B over A"))

                if j in resting_positions and t < len(path_a):
                    if path_a[t] == resting_positions[j]:  # Path A over resting position of B
                        conflicts.append((agents[i], agents[j], path_a[t], t, "Goal Overlap Conflict - A over B"))

        return conflicts

    def diagonalizePath(self, path):

        if len(path) < 3:
            return path  # No staircase possible with less than 3 points

        straightened_path = [path[0]]

        i = 1
        while i < len(path) - 1:
            x1, y1 = path[i - 1] 
            x2, y2 = path[i]     
            x3, y3 = path[i + 1]

            if (x1 == x2 and y2 == y3) or (y1 == y2 and x2 == x3):
                
                i += 1
            else:
                straightened_path.append(path[i])

            i += 1

        straightened_path.append(path[-1])

        return straightened_path
    
class PrioritizedMAPF:

    def __init__(self, directorIn, environmentIn):
        self.director = directorIn
        self.environment = environmentIn
        self.paths = []

    def begin(self, agentsIn):
        # Distance between agent and goal
        for agent in agentsIn:
            distance = self.euclideanDistance(agent.get_location(), agent.get_goal())
            agent.distance = distance
            
        # Order agents by decreasing distance
        shallow_copy = copy.copy(agentsIn)
        shallow_copy.sort()

        for i in range(len(shallow_copy)):
            shallow_copy[i].setPriority(i)

        # Calculate
        reservations = {}
        edge_reservations = {}

        for agent in shallow_copy:
            path = self.calculatePathAStar(agent, reservations, edge_reservations)
            agent.set_path(path)
            self.paths.append(path)

            for t in range(len(path)):
                # Node reservations
                if path[t] in reservations:
                    reservations[path[t]].append(t)
                else:
                    reservations[path[t]] = [t]

                # Edge reservation
                if t < len(path) - 1:
                    # edge = (path[t], path[t + 1])
                    reverse_edge = (path[t + 1], path[t])

                    # Reserve reverse direction
                    if reverse_edge not in edge_reservations:
                        edge_reservations[reverse_edge] = [t]
                    else:
                        edge_reservations[reverse_edge].append(t)
                        
    def calculatePathAStar(self, agent, restrictions, edge_restrictions):
        queue = []
        scene_resolution = self.environment.grid_res
        scene = [[None for _ in range(scene_resolution[1])] for _ in range(scene_resolution[0])]
        start = agent.get_location()
        goal = agent.get_goal()

        # Initialize the starting and goal nodes
        starting_node = self.AStarNode(start)
        starting_node.set_local_score(0)
        starting_node.set_global_score(self.euclideanDistance(start, goal))
        scene[start[0]][start[1]] = starting_node

        goal_node = self.AStarNode(goal)
        scene[goal[0]][goal[1]] = goal_node
        
        heapq.heappush(queue, (starting_node.get_global_score(), starting_node, 0))  # Add time to the queue

        def check_node(check_node, current_node, goal_loc, current_time):
            new_local_score = current_node.get_local_score() + 1
            if check_node.get_local_score() is None or new_local_score < check_node.get_local_score():
                check_node.set_parent(current_node)
                check_node.set_local_score(new_local_score)
                check_node.set_global_score(self.euclideanDistance(check_node.get_coordinates(), goal_loc) + new_local_score)

        while queue:
            _, current_node, current_time = heapq.heappop(queue)
            x, y = current_node.get_coordinates()

            if current_node == goal_node:
                break  # Goal reached

            if current_node.get_visited():
                continue

            # Find neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # LEFT, RIGHT, TOP, BOTTOM
                nx, ny = x + dx, y + dy
                next_time = current_time + 1
                if 0 <= nx < scene_resolution[0] and 0 <= ny < scene_resolution[1] and self.environment.isOccupied((nx, ny)) != 1:
                    # Check for node restrictions
                    if (nx, ny) in restrictions and next_time in restrictions[(nx, ny)]:
                        continue

                    edge = ((x, y), (nx, ny))
                    # reverse_edge = ((nx, ny), (x, y))

                    if edge in edge_restrictions and current_time in edge_restrictions[edge]:
                        continue

                    neighbor_node = scene[nx][ny]
                    if neighbor_node is None:
                        neighbor_node = self.AStarNode((nx, ny))
                        scene[nx][ny] = neighbor_node

                    if not neighbor_node.get_visited():
                        check_node(neighbor_node, current_node, goal, next_time)
                        heapq.heappush(queue, (neighbor_node.get_global_score(), neighbor_node, next_time))

            current_node.set_visited(True)

        # Trace back the path
        path = []
        cursor = goal_node
        while cursor:
            path.append(cursor.get_coordinates())
            cursor = cursor.get_parent()

        path.reverse()
        # path = self.director.diagonalizePath(path)
        return path

    class AStarNode:

        def __init__(self, loc):
            self.coordinates = loc
            self.parent = None
            self.global_score = None
            self.local_score = None
            self.neighbours = [] 
            self.visited = False 

        def __lt__(self, other):
            return self.coordinates < other.coordinates

        def set_visited(self, state):
            self.visited = state
        
        def set_parent(self, parent):
            self.parent = parent

        def set_global_score(self, score):
            self.global_score = score

        def set_local_score(self, score):
            self.local_score = score

        def get_coordinates(self):
            return self.coordinates
        
        def append_neighbour(self, node):
            self.neighbours.append(node)

        def get_neighbours(self):
            return self.neighbours
        
        def get_visited(self):
            return self.visited
        
        def get_global_score(self):
            return self.global_score

        def get_local_score(self):
            return self.local_score
        
        def get_parent(self):
            return self.parent

    def euclideanDistance(self, p1, p2):
        x_distance = (p1[0]-p2[0])**2
        y_distance = (p1[1]-p2[1])**2

        return math.sqrt(x_distance + y_distance)