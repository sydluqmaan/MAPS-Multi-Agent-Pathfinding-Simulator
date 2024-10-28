import pygame
import config as cfg
import math
import time
import pygame.gfxdraw

class Renderer:
    def clamp(n, smallest, largest): 
        return max(smallest, min(n, largest))

    def __init__(self, screen):
        self.sc = screen
        self.screen_width, self.screen_height = screen.get_size()
        self.render_queue = []

    def rect(self, width, height, x_loc, y_loc):
        shape = pygame.Surface((width, height))

        #Hatch mask expose
        mask = pygame.Surface((width, height))
        mask.fill(cfg.BLACK)
        mask.set_colorkey(cfg.BLACK)
        pygame.draw.rect(mask, cfg.WHITE, (0, 0, width, height))
        masked_pattern = pygame.Surface((width, height))
        masked_pattern.blit(self.hatch_layer, (0, 0), area=(x_loc, y_loc, width, height))
        masked_pattern.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        shape.blit(masked_pattern, (0, 0))

        #Draw border
        pygame.draw.rect(shape, cfg.WHITE, shape.get_rect(), 1) #Border width 1

        self.sc.blit(shape, ((800 - width) // 2, (800 - height) // 2))

    def renderToScreen(self):
        self.sc.fill(cfg.BLACK)
        
        for item in self.render_queue:
            surface = item.get_surface()
            location = item.get_loc()
            if (surface != None) and (location != None):
                self.sc.blit(surface, location)
        pygame.display.flip()

    def add_to_render_queue(self, item):
        self.render_queue.append(item)

    def remove_from_render_queue(self, item):
        self.render_queue.remove(item)

class SceneComponent:
    def get_surface(self):
        pass

    def get_loc(self):
        pass

class GUI_Component(SceneComponent):
    def get_size(self):
        pass

class Text(GUI_Component):
    def __init__(self, textIn, sizeIn, colorIn, offsetIn, originIn):
        self.size = sizeIn
        self.text = textIn
        self.color = colorIn
        self.text_surface = self.generateSurface()
        self.offset = offsetIn
        self.loc = (0,0)
        self.calculateLocation(originIn)

    def calculateLocation(self, originIn):
        self.loc = (originIn[0] + self.offset[0], originIn[1] + self.offset[1])

    def set_value(self, textIn):
        self.text = textIn
        self.text_surface = self.generateSurface()

    def set_location(self, locIn):
        self.loc = locIn

    def set_color(self, colorIn):
        self.color = colorIn
        self.text_surface = self.generateSurface()
    
    def get_loc(self):
        return self.loc

    def get_size(self):
        return self.text_surface.get_size()

    def get_surface(self):
        return self.text_surface
    
    def generateSurface(self):
        font = pygame.font.Font(cfg.font1, self.size)
        text_surface = font.render(self.text, True, self.color)

        return text_surface

class AgentInformation(SceneComponent):

    def __init__ (self, idIn, colorIn, offsetIn, originIn, agentIn):
        self.id = idIn
        self.color = colorIn
        self.size = 20
        self.loc = (0,0)
        self.offset = offsetIn
        self.calculateLocation(originIn)
        self.status = "INITIALIZED"
        self.travelTime = "--"
        self.priority = "--"
        self.agent = agentIn

    def calculateLocation(self, originIn):
        self.loc = (originIn[0] + self.offset[0], originIn[1] + self.offset[1])

    def setStatus(self, statusIn):
        self.priority = self.agent.priority
        self.status = statusIn

    def setTravelTime(self, timeIn):
        self.travelTime = timeIn

    def get_surface(self):
        font = pygame.font.Font(cfg.font1, 15)

        id_text = font.render(self.id, True, cfg.WHITE)
        id_text_rect = id_text.get_rect()

        statusElement = font.render(self.status, True, cfg.WHITE)
        statusRect = statusElement.get_rect()

        travelTimeElement = font.render(self.travelTime, True, cfg.WHITE)
        travelTimeRect = travelTimeElement.get_rect()

        # New text elements
        priorityLabelElement = font.render("|   P: ", True, cfg.WHITE)
        priorityLabelRect = priorityLabelElement.get_rect()

        priorityValue = font.render(str(self.priority), True, cfg.WHITE)
        priorityValueRect = priorityValue.get_rect()

        square_surface = pygame.Surface((self.size, self.size))
        square_surface.fill(self.color)

        text_x = (self.size - id_text_rect.width) // 2
        text_y = (self.size - id_text_rect.height) // 2
        square_surface.blit(id_text, (text_x, text_y))

        total_width = (self.size + statusRect.width + travelTimeRect.width + priorityLabelRect.width + priorityValueRect.width + 45)
        total_height = max(self.size, statusRect.height, travelTimeRect.height, priorityLabelRect.height, priorityValueRect.height)

        combined_surface = pygame.Surface((total_width, total_height), pygame.SRCALPHA)

        combined_surface.blit(square_surface, (0, (total_height - self.size) // 2))

        status_x = self.size + 10
        status_y = (total_height - statusRect.height) // 2
        combined_surface.blit(statusElement, (status_x, status_y))

        eta_x = status_x + statusRect.width + 10
        eta_y = (total_height - travelTimeRect.height) // 2
        combined_surface.blit(travelTimeElement, (eta_x, eta_y))

        priorityLabelX = eta_x + travelTimeRect.width + 25
        priorityLabelY = (total_height - priorityLabelRect.height) // 2
        combined_surface.blit(priorityLabelElement, (priorityLabelX, priorityLabelY))

        priorityValueX = priorityLabelX + priorityLabelRect.width + 0
        priorityValueY = (total_height - priorityValueRect.height) // 2
        combined_surface.blit(priorityValue, (priorityValueX, priorityValueY))

        return combined_surface

    def get_loc(self):
        return self.loc
    
class Barrier(SceneComponent):
        def __init__(self, queueIn, screenIn, boundaryIn):
            self.queue = queueIn
            self.screen_width, self.screen_height = screenIn.get_size()
            self.final_layer = None
            self.create_hatch_layer()
            self.boundary = boundaryIn

        def create_hatch_layer(self):
            spacing = 12
            self.hatch_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
            self.hatch_layer.fill(cfg.BLACK)

            # Draw lines from left to bottom and right to top
            for y in range(0, self.screen_height, spacing):  # spacing 10
                pygame.draw.line(self.hatch_layer, cfg.WHITE_A, (0, y), (self.screen_width, y - self.screen_width), 1)
                pygame.draw.line(self.hatch_layer, cfg.WHITE_A, (0, y), (self.screen_width, y + self.screen_width), 1)

            # Draw lines from top to right and bottom to left
            for x in range(0, self.screen_width, spacing):  # spacing 10
                pygame.draw.line(self.hatch_layer, cfg.WHITE_A, (x, 0), (x - self.screen_height, self.screen_height), 1)
                pygame.draw.line(self.hatch_layer, cfg.WHITE_A, (x, 0), (x + self.screen_height, self.screen_height), 1)

        def get_loc(self):
            return self.boundary.get_loc()

        def get_surface(self):

            canvas_res = 1000
            canvas_scaling = self.boundary.get_sim_width()/canvas_res
            # Create the barrier layer and the inflated barrier layer
            barrier_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
            barrier_layer.fill((0, 0, 0, 0))
            inflated_barrier_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
            inflated_barrier_layer.fill((0, 0, 0, 0))

            for component in self.queue:
                if (component.type == "Barrier"):
                    x, y = component.x*canvas_scaling, component.y*canvas_scaling
                    width, height = component.width*canvas_scaling, component.height*canvas_scaling

                    # Draw the rectangle in the barrier layer
                    pygame.draw.rect(barrier_layer, cfg.WHITE, (x, y, width, height))

                    # Draw the inflated rectangle in the inflated barrier layer
                    inflated_rect = pygame.Rect(x, y, width, height).inflate(2, 2)
                    pygame.draw.rect(inflated_barrier_layer, cfg.WHITE, inflated_rect)

            # Create a mask from the barrier layer
            mask_surface = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
            mask_surface.fill((0, 0, 0, 0))
            mask_surface.blit(barrier_layer, (0, 0))

            # Apply the mask to the hatching pattern
            masked_hatch_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
            masked_hatch_layer.blit(self.hatch_layer, (0, 0))
            masked_hatch_layer.blit(mask_surface, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)

            # Create the outline by subtracting the original barrier layer from the inflated barrier layer
            outline_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
            outline_layer.blit(inflated_barrier_layer, (0, 0))
            outline_layer.blit(barrier_layer, (0, 0), special_flags=pygame.BLEND_RGBA_SUB)

            # Combine the masked hatching layer and the outline layer
            self.final_layer = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
            self.final_layer.blit(masked_hatch_layer, (0, 0))
            self.final_layer.blit(outline_layer, (0, 0))

            return self.final_layer

class EndPoint(SceneComponent):
    
    def __init__(self, locIn, colorIn, simIn, envIn):
        self.loc = (locIn[1], 100 - locIn[0]) #use grid res value in place of 100
        self.convertedLocation = ()
        self.color = colorIn
        self.sim = simIn
        self.size = ()
        self.env = envIn
        self.handleResize()
        self.surface = self.generateSurface()

    def generateSurface(self):
        rect_surface = pygame.Surface((self.size[0], self.size[1]), pygame.SRCALPHA)

        colorA = (self.color[0], self.color[1], self.color[2], 75)
        
        # Fill
        fill_rect = pygame.Rect(1, 1, self.size[0], self.size[1])
        pygame.draw.rect(rect_surface, colorA, fill_rect)

        # Border
        outer_rect = pygame.Rect(0, 0, self.size[0], self.size[1])
        pygame.draw.rect(rect_surface, self.color, outer_rect, 1)

        return rect_surface
    
    def get_surface(self):
        return self.surface
    
    def get_loc(self):
        return self.convertedLocation
    
    def handleResize(self):
        self.size = (self.sim.unit_width,self.sim.unit_width)
        convertedLocation = self.sim.envToScreenCoordinates(self.loc)
        
        selfOffset = self.size[0]/2

        x = convertedLocation[0] - selfOffset
        y = convertedLocation[1] - (selfOffset * 3)
        self.convertedLocation = (x,y)

        self.surface = self.generateSurface()

class RenderAgent(SceneComponent):
    
    def __init__ (self, idIn, colorIn, sizeIn, locIn, envIn, simulationIn):
        self.id = idIn
        self.color = colorIn
        self.sim = simulationIn
        self.env = envIn
        self.size = sizeIn
        self.path = None
        self.screenPath = []
        self.activeGoalNode = -1
        self.activeGoalLoc = None
        self.loc = locIn
        self.scaling = None
        self.loc = self.getInitialPosition((locIn[1], 100 - locIn[0]))
        self.timer = TimeTracker()
        self.agentInformationElement = None
        self.surface = self.generateSurface()

    def getInitialPosition(self, coordinatesIn):
        self.scaling = self.sim.get_sim_width()
        
        convertedLocation = self.sim.envToScreenCoordinates(coordinatesIn)
        
        selfOffset = self.size/2

        x = convertedLocation[0] - (selfOffset * 2)
        y = convertedLocation[1] - (selfOffset * 3)

        location = (x,y)

        return location

    def get_loc(self):
        return self.loc

    def generateSurface(self):
        font = pygame.font.Font(cfg.font1, 12)
        text = font.render(self.id, True, cfg.WHITE)
        text_rect = text.get_rect()

        square_surface = pygame.Surface((self.size, self.size))
        square_surface.fill(self.color)

        text_x = (self.size - text_rect.width) // 2
        text_y = (self.size - text_rect.height) // 2
        square_surface.blit(text, (text_x, text_y))

        # Triangle marker
        triangle_size = int(self.size * 0.35)  #35 percent
        triangle_points = [
            (self.size - triangle_size, self.size),  # Bottom-left point of the triangle
            (self.size, self.size - triangle_size),  # Top-right point of the triangle
            (self.size, self.size)  # Bottom-right corner of the square
        ]

        # Draw triangle
        pygame.draw.polygon(square_surface, cfg.WHITE, triangle_points)

        return square_surface

    def get_surface(self):
        return self.surface

    def pauseTimer(self):
        self.timer.pause()

    def resumeTimer(self):
        if self.activeGoalNode != -1:
            self.timer.resume()

    def set_active_goal_node(self, nodeId):
        self.activeGoalLoc = self.screenPath[nodeId]
        self.activeGoalNode = nodeId

        if nodeId == 1:
            self.timer.start()
            self.agentInformationElement.setStatus("MOVING")

        elif nodeId == -1:
            self.timer.pause()
            self.agentInformationElement.setStatus("REACHED")
            self.agentInformationElement.setTravelTime(str(self.timer.get_elapsed_time()) + "s")

    def handleResize(self):

        selfOffset = self.size / 2
        scalingFactor = self.sim.get_sim_width() / self.scaling
        self.scaling = self.sim.get_sim_width()

        x = (self.loc[0] + selfOffset) * scalingFactor - selfOffset
        y = (self.loc[1] + (selfOffset * 2)) * scalingFactor - (selfOffset * 2)
        self.loc = (x, y)

        if self.activeGoalNode != -1:
            self.convertNodes()
            if 0 <= self.activeGoalNode < len(self.screenPath):
                self.activeGoalLoc = self.screenPath[self.activeGoalNode]
        
    def convertNodes(self):
        self.screenPath.clear()
        for nodeID in range(0,len(self.path)):

            x = self.sim.get_sim_width() - ( (self.path[nodeID][0] * self.sim.unit_width) + round(self.sim.unit_width/2))
            y = (self.path[nodeID][1] * self.sim.unit_width) + round(self.sim.unit_width/2)

            self.screenPath.append((round(y,2),round(x,2)))

class Simulation(SceneComponent):

    def __init__(self, screenIn, rendererIn, envIn):
        self.screen = screenIn
        self.simulationAreaDimensions = None
        self.simulationAreaOrigin = None
        self.grid_overlay = False
        self.renderer = rendererIn
        self.env = envIn
        self.agentsDict = {}
        self.agentSpeed = 1
        self.time_stamp = time.time()
        self.activePathInstances = []
        self.endPoints = []
        self.handleResize()
        self.unit_width = self.get_sim_width() / self.env.grid_res[0]
        self.stepQueue = []
        self.completedQueue = []
        self.panel = None
        self.state = "INITIALIZED"

    def setSimulationState(self, stateIn):
        match stateIn:
            case "ACTIVE":
                if self.getSimulationState() == "PAUSED":
                    for renderAgent in self.agentsDict:
                        renderAgent.resumeTimer()

            case "PAUSED":
                for renderAgent in self.agentsDict:
                        renderAgent.pauseTimer()

            case "COMPLETED":
                self.panel.set_sim_state("COMPLETED")

        self.state = stateIn

    def getSimulationState(self):
        return self.state
    
    def resetSimulation(self):
        for renderAgent in self.agentsDict:
            self.renderer.remove_from_render_queue(renderAgent)

        for endpoint in self.endPoints:
            self.renderer.remove_from_render_queue(endpoint)

        for renderPath in self.activePathInstances:
            self.renderer.remove_from_render_queue(renderPath)

        self.agentsDict = {}
        self.activePathInstances = []
        self.stepQueue = []
        self.endPoints = []
        self.completedQueue = []

    def handleResize(self):
        margin = 15
        screenWidth, screenHeight = self.screen.get_size()
        simulationAreaHeight = round((screenHeight - (margin*2)),-1)
        simulationAreaWidth = simulationAreaHeight
        simulationAreaOriginX = margin
        simulationAreaOriginY = margin

        self.unit_width = simulationAreaWidth / self.env.grid_res[0]
        self.simulationAreaDimensions = (simulationAreaWidth, simulationAreaHeight)
        self.simulationAreaOrigin = (simulationAreaOriginX, simulationAreaOriginY)

        for path in self.activePathInstances:
            path.handleResize()

        for endPoint in self.endPoints:
            endPoint.handleResize()

        for displayAgent in self.agentsDict:
            displayAgent.handleResize()

    def getSimulationAreaDimensions(self):
        return self.simulationAreaDimensions
    
    def getSimulationAreaOrigin(self):
        return self.simulationAreaOrigin
    
    def envToScreenCoordinates(self, coordinatesIn):
        # Calculate the center offset for a grid unit
        unit_offset = self.unit_width / 2

        # Calculate the origin coordinates of simulation area
        simulationOrigin = self.getSimulationAreaOrigin()

        x = simulationOrigin[0] + (coordinatesIn[0] * self.unit_width) + unit_offset 
        y = simulationOrigin[1] + (coordinatesIn[1] * self.unit_width) + unit_offset 

        location = (x, y)

        return location

    def toggle_grid_overlay(self, state):
        self.grid_overlay = state

    def get_sim_width(self):
        return self.getSimulationAreaDimensions()[0]

    def get_surface(self):

        screen_width, screen_height = self.screen.get_size()
        border_surface = pygame.Surface((screen_width, screen_height), pygame.SRCALPHA)

        border_surface.fill((0, 0, 0, 0))

        border_thickness = 1

        simulationAreaDimensions = self.getSimulationAreaDimensions()
        spacing_x = simulationAreaDimensions[0] / self.env.grid_res[0]
        spacing_y = simulationAreaDimensions[1] / self.env.grid_res[1]

        # Draw border
        pygame.draw.rect(border_surface, cfg.WHITE, (0, 0, simulationAreaDimensions[0], simulationAreaDimensions[1]), border_thickness)

        if self.grid_overlay == True:
            for i in range(1, self.env.grid_res[0]):
                x = int(i * spacing_x)
                pygame.draw.line(border_surface, cfg.RED, (x, self.getSimulationAreaOrigin()[1]-15), (x, simulationAreaDimensions[1]), 1)

            for j in range(1, self.env.grid_res[1]):
                y = int(j * spacing_y)
                pygame.draw.line(border_surface, cfg.RED, (0, y), (simulationAreaDimensions[0], y), 1)

        return border_surface

    def get_loc(self):
        return self.getSimulationAreaOrigin()

    def getAgentDict(self):
        return self.agentsDict

    def init_agents(self, agentsIn):

        for agent in agentsIn:
            agentDisplayElement = RenderAgent(agent.id, agent.color, 15, agent.get_location(), self.env, self)
            self.agentsDict[agentDisplayElement] = agent
            self.renderer.add_to_render_queue(agentDisplayElement)
            endPoint = EndPoint(agent.get_goal(), agent.color,  self, self.env)
            self.renderer.add_to_render_queue(endPoint)
            self.endPoints.append(endPoint)
    
    def init_paths(self):
        for agentDisplay in self.agentsDict:

            path = self.agentsDict[agentDisplay].get_path()
            agentDisplay.path = path
            agentDisplay.convertNodes()

            if len(path) > 1:
                agentDisplay.set_active_goal_node(1)
                self.stepQueue.append(agentDisplay)
            
            path_instance = Path(self.getSimulationAreaOrigin(), self.unit_width, path, agentDisplay.color, self.screen, self)
            self.activePathInstances.append(path_instance)
            
        for path_instance in self.activePathInstances:
            self.renderer.add_to_render_queue(path_instance)

    def time_step(self):
        current_time = time.time()
        
        # Compute elapsed time
        time_elapsed = current_time - self.time_stamp

        self.time_stamp = current_time
        
        for agentDisplayElement in self.stepQueue:

            if agentDisplayElement.activeGoalNode != -1:
                current_position = agentDisplayElement.get_loc()
                target = agentDisplayElement.activeGoalLoc

                speed = 50

                dx = target[0] - (current_position[0])
                dy = target[1] - (current_position[1])

                distance = math.sqrt(dx**2 + dy**2)
                if distance > 0:
                    direction = (dx / distance, dy / distance)

                    movement = speed * time_elapsed

                    newX = current_position[0] + (direction[0] * movement)
                    newY = current_position[1] + (direction[1] * movement)

                    if distance < movement:
                        agentDisplayElement.loc = target
                        if agentDisplayElement.activeGoalNode + 1 < len(agentDisplayElement.path):
                            agentDisplayElement.set_active_goal_node(agentDisplayElement.activeGoalNode + 1)
                        else:
                            agentDisplayElement.set_active_goal_node(-1)
                            self.completedQueue.append(agentDisplayElement)
                            self.stepQueue.remove(agentDisplayElement)

                            if len(self.stepQueue) == 0:
                                self.setSimulationState("COMPLETED")
                                makespan = 0
                                flowtime = 0
                                for renderAgent in self.completedQueue:
                                    elapsedTime = renderAgent.timer.get_elapsed_time()
                                    flowtime += elapsedTime
                                    if elapsedTime > makespan:
                                        makespan = elapsedTime
                                self.panel.setMakespan(makespan)
                                self.panel.setFlowTime(round(flowtime,2))

                    else:
                        agentDisplayElement.loc = (newX, newY)
                        self.panel.setElapsedTime()

class Panel():

    def __init__ (self, simIn, screenIn, rendererIn):
        self.simulation = simIn
        self.screen = screenIn
        self.origin = self.calculate_origin()
        self.renderer = rendererIn
        self.textElements = []
        self.informationElements = []

        self.timer = TimeTracker()

        self.column1 = Text("Simulation ", 18, cfg.WHITE, (0, 0), self.origin)
        self.initializeElement(self.column1)

        self.Label9 = Text("Simulation Status: ", 18, cfg.WHITE, (0,125), self.origin)
        self.initializeElement(self.Label9)

        self.Label10 = Text("INITIALIZED", 18, cfg.WHITE, (190,125), self.origin)
        self.initializeElement(self.Label10)

        self.Label1 = Text("Agents active: ", 18, cfg.WHITE, (0,25), self.origin)
        self.initializeElement(self.Label1)

        self.Label4 = Text("0", 18, cfg.WHITE, (150,25), self.origin)
        self.initializeElement(self.Label4)

        self.Label2 = Text("Grid Overlay: ", 18, cfg.WHITE, (0,100), self.origin)
        self.initializeElement(self.Label2)

        self.Label3 = Text("OFF", 18, cfg.RED, (150,100), self.origin)
        self.initializeElement(self.Label3)

        self.algorithmLabel = Text("Pathfinding Algorithm: A* ", 18, cfg.WHITE, (0,75), self.origin)
        self.initializeElement(self.algorithmLabel)

        self.Label7 = Text("Grid Resolution: ", 18, cfg.WHITE, (0,50), self.origin)
        self.initializeElement(self.Label7)

        self.Label8 = Text("0x0", 18, cfg.WHITE, (175,50), self.origin)
        self.initializeElement(self.Label8)

        self.column2 = Text("Metrics ", 18, cfg.WHITE, (330, 0), self.origin)
        self.initializeElement(self.column2)

        self.Label5 = Text("Makespan: ", 18, cfg.WHITE, (330,25), self.origin)
        self.initializeElement(self.Label5)

        self.makeSpanValueElement = Text("--", 18, cfg.WHITE, (435,25), self.origin)
        self.initializeElement(self.makeSpanValueElement)

        self.Label6 = Text("Flowtime: ", 18, cfg.WHITE, (330,50), self.origin)
        self.initializeElement(self.Label6)

        self.flowTimeValueElement = Text("--", 18, cfg.WHITE, (435,50), self.origin)
        self.initializeElement(self.flowTimeValueElement)

        self.Label11 = Text("Conflicts resolved:", 18, cfg.WHITE, (330,75), self.origin)
        self.initializeElement(self.Label11)

        self.conflictsCountValue = Text("--", 18, cfg.WHITE, (530,75), self.origin)
        self.initializeElement(self.conflictsCountValue)
        
        self.elapsedTimeLabel = Text("Time Elapsed:", 18, cfg.WHITE, (330,100), self.origin)
        self.initializeElement(self.elapsedTimeLabel)

        self.elapsedTimeValue = Text("--", 18, cfg.WHITE, (475,100), self.origin)
        self.initializeElement(self.elapsedTimeValue)

        self.initializeAgentInformation()

    def initializeElement(self, elementIn):
        self.textElements.append(elementIn)
        self.renderer.add_to_render_queue(elementIn)

    def resetPanel(self):
        for element in self.informationElements:
            self.renderer.remove_from_render_queue(element)
        self.informationElements = []
        self.elapsedTimeValue.set_value("--")
        self.timer.reset()
        self.makeSpanValueElement.set_value("--")
        self.flowTimeValueElement.set_value("--")

    def initializeAgentInformation(self):
        agentsDict = self.simulation.getAgentDict()
        offsetY = 200

        for renderAgent in agentsDict:
            agent = agentsDict[renderAgent]
            agentInfoElement = AgentInformation(str(agent.getID()), agent.getColor(), (0,offsetY), self.origin, agent)
            renderAgent.agentInformationElement = agentInfoElement
            self.initializeElement(agentInfoElement)
            self.informationElements.append(agentInfoElement)
            offsetY += 25

    def setElapsedTime(self):
        self.elapsedTimeValue.set_value(str(self.timer.get_elapsed_time()) + "s")

    def handleResize(self):
        for element in self.textElements:
            element.calculateLocation(self.calculate_origin())

    def setMakespan(self, spanIn):
        self.makeSpanValueElement.set_value(str(spanIn)+"s")
        
    def setFlowTime(self, flowTimeIn):
        self.flowTimeValueElement.set_value(str(flowTimeIn)+"s")

    def setConflictCount(self, countIn):
        self.conflictsCountValue.set_value(str(countIn))

    def set_agent_num(self, numIn):
        self.Label4.set_value(str(numIn))

    def setGridResolution(self, resolution):
        self.Label8.set_value(str(resolution[0]) + " X " + str(resolution[1]))

    def set_sim_state(self, stateIn):
        self.Label10.set_value(str(stateIn))

        match stateIn:
            case "INITIALIZED":
                self.Label10.set_color(cfg.WHITE)
                self.timer.reset()

            case "ACTIVE":
                self.Label10.set_color(cfg.GREEN)
                if self.timer.is_paused:
                    self.timer.resume()
                else:
                    self.timer.start()

            case "PAUSED":
                self.Label10.set_color(cfg.BRIGHT_RED)
                self.timer.pause()

    def calculate_origin(self):
        simulationAreaWidth, simulationAreaHeight = self.simulation.getSimulationAreaDimensions()

        margin = 15
        
        x = margin + simulationAreaWidth + margin
        y = margin

        return (x,y)

    def toggle_grid_overlay(self, stateIn):
        if stateIn:
            self.Label3.set_value("ON")
            self.Label3.set_color(cfg.GREEN)
        else:
            self.Label3.set_value("OFF")
            self.Label3.set_color(cfg.RED)

class Path(SceneComponent):

    def __init__(self, locIn, tileSizeIn, pointsIn, colorIn, screenIn, simulationIn):
        self.loc = locIn
        self.tileSize = tileSizeIn
        self.points = pointsIn
        self.convertedPoints = []
        self.color = colorIn
        self.simulation = simulationIn
        self.screen_width, self.screen_height = screenIn.get_size()
        self.handleResize()
    
    def generateSurface(self):
        self.path_surface = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
        previous_point = self.convertedPoints[0]
        for i in range(1,len(self.convertedPoints)):

            pygame.draw.line(self.path_surface, self.color, previous_point, self.convertedPoints[i], 1)

            previous_point = self.convertedPoints[i]

    def get_surface(self):
        return self.path_surface

    def get_loc(self):
        return self.loc
    
    def handleResize(self):
        self.convertedPoints.clear()
        for i in range(0, len(self.points)):
            x = self.simulation.get_sim_width() - ( (self.points[i][0] * self.simulation.unit_width) + round(self.simulation.unit_width/2))
            y = (self.points[i][1] * self.simulation.unit_width) + round(self.simulation.unit_width/2)

            self.convertedPoints.append((round(y,2),round(x,2)))
        self.generateSurface()

class TimeTracker:
    def __init__(self):
        self.start_time = 0.0
        self.pause_time = 0.0
        self.is_paused = False
        self.total_elapsed = 0.0

    def start(self):
        if self.is_paused:
            self.is_paused = False
            self.start_time = time.time() - (self.pause_time - self.start_time)
        else:
            self.start_time = time.time()
            self.total_elapsed = 0.0

    def pause(self):
        if not self.is_paused:
            self.pause_time = time.time()
            self.total_elapsed += self.pause_time - self.start_time
            self.is_paused = True

    def resume(self):
        if self.is_paused:
            self.is_paused = False
            self.start_time = time.time()

    def reset(self):
        self.start_time = time.time()
        self.pause_time = 0.0
        self.is_paused = False
        self.total_elapsed = 0.0

    def get_elapsed_time(self):
        if self.is_paused:
            return round(self.total_elapsed, 2)
        else:
            return round(self.total_elapsed + (time.time() - self.start_time), 2)