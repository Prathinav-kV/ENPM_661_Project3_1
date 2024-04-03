import numpy as np
import cv2
from queue import PriorityQueue
import time
from datetime import datetime
import pygame
import sys

d = 0
ctc = 0
weight = 1

def valid_point(x,y):
    
    equations = hexagon_side_equations()
    
    if ( (x >= d) and (x<=(1200-(d))) and (y<=(500-(d))) and (y>=(d)) ):
        # print( " Inside box ")
        if not ( (x>(100-(d))) and (x<(175+(d))) and (y > (100-(d)))) :
            # print(" Outside redtangle 1")
            if not ( (x>(275-(d))) and (x<(350+(d))) and (y<(400+(d))) ):
                # print(" Outside redtangle 2")
                if not ( ( ( y - equations[0][0] * x - equations[0][1] ) < 0 ) and (x>equations[1][0]) and ( ( y - equations[2][0] * x - equations[2][1]) > 0) 
                        and ( ( y - equations[3][0]*x - equations[3][1]) > 0 ) and ( x < equations[4][0]) and ( ( y - equations[5][0]*x - equations[5][1]) < 0 ) ):
                    # print(" Outside hexagon")
                    if not ( (x>(900-d)) and (x<(1100+d)) and (y<(450+d)) and (y>(50-d)) ):
                        # print(" Outside d shaped objedt")
                        return True
                    else:
                        if ( (y<(375-d)) and (y>(125+d)) and (x<(1020-d)) ):
                            # print(" Safe zone of the C shaped object")
                            return True
                        else:
                            # print(" Inside the C shaped object")
                            return False
                else:
                    # print(" Inside Hexagon")
                    return False
            else:
                # print(" Inside rectangle 2")
                return False
        else:
            # print(" Inside rectangle 1")
            return False
    else:
        # print(" Outside box ")
        return False
        
from math import cos, sin, radians

def hexagon_side_equations():
    # Constants
    Cx, Cy = 650, 250  # Center of the hexagon
    R = 150 + np.sqrt(2)*d  # Updated side length
    start_angle = 90  # Starting angle
    
    # Calculate vertices of the hexagon
    vertices = [((Cx + R * cos(radians(start_angle + 60 * i))), ( Cy + R * sin(radians(start_angle + 60 * i)))) for i in range(6)]
    
    # Function to calculate the equation of a line given two points
    def line_equation(x1, y1, x2, y2):
        if int(x2) != int(x1):
            m = (y2 - y1) / (x2 - x1)  # Slope
            b = y1 - m * x1  # Intercept
            # return (round(m,2), round(b,2))
            return (m,b)
        else:
            # If the line is vertical, return None for slope and x-coordinate of the line
            return (x1,None)
    
    # Calculate equations for each side of the hexagon
    equations = []
    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]  # Wrap around to the first vertex
        equations.append(line_equation(x1, y1, x2, y2))

    return equations

def move_star(node):
    node_x = node[0]
    node_y = node[1]
    node_theta = node[2]*30
    
    newNodes = [] # new node information

    for i in range(-2,3):
        new_theta = (node_theta + i*30) % 360  # Ensure angle stays within range 0 to 360 degrees
        new_theta_rad = np.radians(new_theta)
        x = round(node_x + ctc * np.cos(new_theta_rad))
        y = round(node_y + ctc * np.sin(new_theta_rad))
        if not valid_point(x,y):
            continue
        newNodes.append((x,y,new_theta//30))

    return newNodes

def inGoal(node, goal_node):
    goal_radius = 3
    x_goal = goal_node[0]
    y_goal = goal_node[1]
    theta_goal = goal_node[2]
    x_node = node[0]
    y_node = node[1]
    theta_node = node[2]
    return np.sqrt(np.square(x_node-x_goal) + np.square(y_node-y_goal)) < goal_radius and theta_node == theta_goal

def heuristic(node, goal_node):
  return weight * np.sqrt(np.square(goal_node[0] - node[0]) + np.square(goal_node[1] - node[1]))

def a_star_algorithm(start, goal):

    # Create cost_grid and initialize cost to come for start_node
    cost_grid = [[[float('inf')] * 12 for _ in range(500)] for _ in range(1200)]
    cost_grid[start[0]][start[1]][start[2]] = 0

    # Create grid to store parents
    parent_grid = [[[None] * 12 for _ in range(500)] for _ in range(1200)]
    parent_grid[start[0]][start[1]][start[2]] = None

    # Create grid to store parents
    visited_grid = [[[False] * 12 for _ in range(500)] for _ in range(1200)]
    visited_list = []

    # Priority queue to store open nodes
    # Cost to come, coordinate values (x,y), parent
    open_queue = PriorityQueue()
    open_queue.put((0, start))  # (priority, node)
    
    open = {start:(0,start)}

    while open_queue:
        element = open_queue.get()
        node = element[1]
        del open[node]
        visited_grid[node[0]][node[1]][node[2]//30] = True
        visited_list.append(node)

        if inGoal(node, goal):
            return parent_grid, visited_list

        # Get neighboring nodes
        actions = move_star(node)
        node_cost = cost_grid[node[0]][node[1]][node[2]]

        for action in actions:
            move = action
            if not visited_grid[move[0]][move[1]][move[2]]:
                new_cost = node_cost + ctc
                if new_cost < cost_grid[move[0]][move[1]][move[2]]:
                    cost_grid[move[0]][move[1]][move[2]] = new_cost
                    priority = new_cost + heuristic(move, goal)
                    open_queue.put((priority, move))                    
                    open[move] = (priority,move)
                    parent_grid[move[0]][move[1]][move[2]] = node

    return parent_grid, visited_list, print("Failed to find goal")

def find_path(parent_grid, visited_list, start):
    current_node = visited_list[-1]
    path = [current_node]
    start_node = start
    while start_node != current_node:
        temp_node = parent_grid[current_node[0]][current_node[1]][current_node[2]]
        current_node = temp_node
        path.insert(0, current_node)
    return path

def animate(start,goal,path,visited_list):
    # Initialize Pygame
    pygame.init()

    # Define colors
    pastel_background = (230, 230, 250)  # Pastel background color
    dark_pastel_background = (200, 200, 220)  # Darker pastel background color
    red = (255, 0, 0)
    black = (0, 0, 0)
    blue = (0,0,255)
    white = (255,255,255)

    # Set canvas dimensions
    canvas_width = 1200
    canvas_height = 500

    # Create the canvas
    canvas = pygame.display.set_mode((canvas_width, canvas_height))
    pygame.display.set_caption("Canvas with Shapes")

    # Define rectangle properties
    rect_width = 75
    rect_height = 400
    rect1 = pygame.Rect(100, 0, rect_width, rect_height)
    rect2 = pygame.Rect(275, 100, rect_width, rect_height)
    rect3 = pygame.Rect(900, 50, 200, 75)
    rect4 = pygame.Rect(1020, 125, 80, 250)
    rect5 = pygame.Rect(900, 375, 200, 75)

    # Main loop
    running = True
    clock = pygame.time.Clock()  # Initialize the clock
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Fill the canvas with pastel background color
        canvas.fill(pastel_background)

        # Draw rectangles
        pygame.draw.rect(canvas, dark_pastel_background, rect1)
        pygame.draw.rect(canvas, dark_pastel_background, rect2)
        pygame.draw.rect(canvas, dark_pastel_background, rect3)
        pygame.draw.rect(canvas, dark_pastel_background, rect4)
        pygame.draw.rect(canvas, dark_pastel_background, rect5)

        # Draw hexagon
        pygame.draw.polygon(canvas, dark_pastel_background, [(650, 100), (520, 175), (520, 325), (650, 400), (780, 325), (780, 175)])

        # Draw points
        pygame.draw.circle(canvas, red, (start[0], 500 - start[1]), 1)
        pygame.draw.circle(canvas, blue, (goal[0], 500 - goal[1]), 1)
        pygame.display.flip()

        for i in visited_list:
            pygame.draw.circle(canvas, white, (int(i[0]), int(500 - i[1])), 1)
            pygame.draw.circle(canvas, red, (start[0], 500 - start[1]), 1)
            pygame.display.flip()

        for i in path:
            pygame.draw.circle(canvas, black, (int(i[0]), int(500 - i[1])), 1)
            pygame.display.flip()
            pygame.time.delay(10)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(1)  # Adjust the frame rate as needed

    # Quit Pygame
    pygame.quit()
    sys.exit()

def main():
    
    global d
    global ctc
    
    
    clear = round(int(input("Enter the clearance in the canvas: ")))
    clear = abs(clear)
    l = int(input("Enter the stride length(0 <= l <= 10) in the canvas l: "))
    l = abs(l)
    radius = int(input("Enter the robot radius: "))
    radius = abs(radius)
    
    d = clear+radius
    ctc = l
    
    start_x = int(input("Enter the starting coordinate x: "))
    start_x = abs(start_x)
    start_y = int(input("Enter the starting coordinate y: "))
    start_y = abs(start_y)
    start_th = (int(input("Enter the starting orientation in degrees: "))%360)//30

    while not (valid_point(start_x,start_y)):
        print("Please try another starting point not in the object area")
        start_x = int(input("Enter the starting coordinate x: "))
        start_y = int(input("Enter the starting coordinate y: "))

        
    goal_x = int(input("Enter the goal coordinate x: "))
    goal_x = abs(goal_x)
    goal_y = int(input("Enter the goal coordinate y: "))
    goal_y = abs(goal_y)
    goal_th = (int(input("Enter the goal orientation: "))%360)//30

    while not (valid_point(goal_x,goal_y)) or ((goal_x == start_x)and(goal_y==start_y)):
        print("Please try another goal point not equal to start point and not in the object area")
        goal_x = int(input("Enter the goal coordinate x: "))
        goal_y = int(input("Enter the goal coordinate y: "))
    
    start = (start_x,start_y,start_th)
    goal = (goal_x,goal_y,goal_th)
    
    # Start timer
    start_time = datetime.now()

    print('Exploring nodes...')
    explored = a_star_algorithm(start, goal)
    parent_grid = explored[0]
    visited_list = explored[1]

    print('Generating path...')
    path = find_path(parent_grid, visited_list, start)

    # Get time taken to find path
    end_time = datetime.now()
    time_taken = end_time - start_time
    
    print('Path found in: ', time_taken," seconds")

    animate(start,goal,path,visited_list)

if __name__=="__main__":
    main()