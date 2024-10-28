import pygame
import sys
import render
import pathfinding
import os 

if getattr(sys, 'frozen', False):
    current_dir = os.path.dirname(sys.executable)
else:
    current_dir = os.path.dirname(__file__)

icon_path = os.path.join(current_dir, 'assets', 'Icon.png')

#Sim Config
simResolution = (100,100)
agentCount = 20
pygame.init()

# Set up the game window
screenInfo = pygame.display.Info()
startScreenWidth = screenInfo.current_w - 150
startScreenHeight = screenInfo.current_h - 150

screen = pygame.display.set_mode((startScreenWidth, startScreenHeight), pygame.RESIZABLE)
pygame.display.set_caption("MAPS")

iconImage = pygame.image.load(icon_path) 
pygame.display.set_icon(iconImage)

#Environment
env = pathfinding.Environment(simResolution)
mapComponents = env.getComponents()

#Renderer
rd = render.Renderer(screen)

#Initialize Simulation Window
simulation = render.Simulation(screen, rd, env)
rd.add_to_render_queue(simulation)

#Initialize barrier layer1
barrier = render.Barrier(mapComponents, screen, simulation)
rd.add_to_render_queue(barrier)

#Initialize director
director = pathfinding.Director(env, simulation)
director.initializeAgents(agentCount)

#Initialize panel
panel = render.Panel(simulation, screen, rd)
panel.set_agent_num(agentCount)
panel.setGridResolution(simResolution)
simulation.panel = panel

def set_state(stateIn):
    current_state = simulation.getSimulationState()
    
    if stateIn == "PAUSED" and (current_state == "INITIALIZED" or current_state == "COMPLETED"):
        return
    
    if stateIn == "ACTIVE" and current_state == "COMPLETED":
        return

    if current_state != stateIn:
        panel.set_sim_state(stateIn)
        
        match stateIn:
            case "ACTIVE":
                if current_state == "INITIALIZED":
                    conflicts = director.initializePaths()
                    panel.setConflictCount(len(conflicts[0]))

        simulation.setSimulationState(stateIn)

def resetSimulation():
    simulation.resetSimulation()
    simulation.setSimulationState("INITIALIZED")
    panel.set_sim_state("INITIALIZED")
    director.resetDirector()
    panel.resetPanel()
    director.initializeAgents(agentCount)
    panel.initializeAgentInformation()
    panel.setConflictCount("--")

while True:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.VIDEORESIZE:
            simulation.handleResize()
            panel.handleResize()

    #Handle inputs
    keys = pygame.key.get_pressed()
    if keys[pygame.K_1]:
        simulation.toggle_grid_overlay(True)
        panel.toggle_grid_overlay(True)
    if keys[pygame.K_2]:
        simulation.toggle_grid_overlay(False)
        panel.toggle_grid_overlay(False)
    if keys[pygame.K_3]:
        set_state("ACTIVE")
    if keys[pygame.K_4]:
        set_state("PAUSED")
    if keys[pygame.K_5]:
        resetSimulation()

    match simulation.getSimulationState():
        case "ACTIVE":
            simulation.time_step()
            pass

        case "PAUSED":
            pass

    rd.renderToScreen()