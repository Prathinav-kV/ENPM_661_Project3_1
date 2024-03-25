import numpy as np
import heapq as hq
import matplotlib.pyplot as plt
from datetime import datetime
import sys
import pygame

def valid_point(x,y,c):
    d = c+5
    equations = hexagon_side_equations(d)
    
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

def hexagon_side_equations(c):
    # Constants
    Cx, Cy = 650, 250  # Center of the hexagon
    R = 150 + np.sqrt(2)*c  # Updated side length
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

def move_star(ctc,x,y,theta,l,c):
    
    angles = [30,60,90,0,-30,-60,-90]
    new_points = []
    for angle in angles:
        new_theta = theta+angle
        while new_theta<0:
            new_theta +=360
        while (new_theta >= 360):
            new_theta -=360
        # x_new = round(x + l*np.cos(np.deg2rad(new_theta)),2)
        # y_new = round(y + l*np.sin(np.deg2rad(new_theta)),2)
        x_new = x + l*np.cos(np.deg2rad(new_theta))
        
        y_new = y + l*np.sin(np.deg2rad(new_theta))
        
        if not valid_point(x_new,y_new,c):
            continue
        new_ctc = ctc + l
        x_new = round(x_new)
        y_new = round(y_new)
        new_points.append([new_ctc,x_new,y_new,new_theta])
    return new_points

clear = int(input("Enter the clearance in the canvas: "))
l = int(input("Enter the stride length(0 <= l <= 10) in the canvas l: "))
start_x = int(input("Enter the starting coordinate x: "))
start_y = int(input("Enter the starting coordinate y: "))
start_th = int(input("Enter the starting orientation in degrees: "))
# while start_th >= 360:
#     start_th -=360
while not (valid_point(start_x,start_y,clear)):
    print("Please try another starting point not in the object area")
    start_x = int(input("Enter the starting coordinate x: "))
    start_y = int(input("Enter the starting coordinate y: "))

    
goal_x = int(input("Enter the goal coordinate x: "))
goal_y = int(input("Enter the goal coordinate y: "))
goal_th = int(input("Enter the goal orientation: "))
# while goal_th >= 360:
#     goal_th -=360
while not (valid_point(goal_x,goal_y,clear)) or ((goal_x == start_x)and(goal_y==start_y)):
    print("Please try another goal point not equal to start point and not in the object area")
    goal_x = int(input("Enter the goal coordinate x: "))
    goal_y = int(input("Enter the goal coordinate y: "))  


# Defining the variables

visited = np.array([0,0]) # visited list
q = []      # the open queue
c = 0       # counter for the nodes



ind_mat = np.ones((2400,1000,12),dtype=int)
ind_mat *= -1

goal_point = 0  # variable to check the goal index
start_element = [-1,0,start_x,start_y,start_th,0] # starting point added as an element to the heapq in the format 
# 0: total_cost, 1: cost, 2: x, 3: y, 4: theta,  5: node index, 6: parent

goal = (goal_x,goal_y,goal_th)
start = (start_x,start_y,start_th)

hq.heappush(q, start_element)
hq.heapify(q)       # creating the heapq

graph = np.matrix([start_x,start_y,start_th,0,-2]) # adding the starting element to the graph which is a numpy matrix 

start_time = datetime.now()  # start time 
while q:
    
    point = hq.heappop(q)
    
    if ((point[2]-goal_x)**2+(point[3]-goal_y)**2)<=0.25:  # checking if the node is in the goal region
        print(" Goal region reached")
        goal_point = (point[2],point[3],point[4])
        break
    
    # performing the movement
    neighbors = move_star(point[1],point[2],point[3],point[4],l,clear)  # getting all the neighboring elements
    
    for n in neighbors:

        ind = ind_mat[n[1]*2,n[2]*2,int(n[3]/30)] # obtaining the index of the existing node of that coordinate
        tot_n = n[0] + np.sqrt(((n[1]-goal_x)**2+(n[2]-goal_y)**2))   # new total of the neighbor
        
        if ind > -1:  # checking if we have encountered it
            
            # Yes we encountered it
            tot_curr = graph[ind,4] + np.sqrt(((graph[ind,0]-goal_x)**2+(graph[ind,1]-goal_y)**2))  # total currently
            
            if tot_n < tot_curr:  # checking if the new total is less than the current total
                # Yes it is lesser
                # We will update the heap and graph with the new total and new parent

                # Updating the graph
                graph[ind,3] = n[0]
                graph[ind,4] = point[5]

                # Updating the Heap
                q_index = -1        # index of the heap element to upgrade

                for i,ele in enumerate(q):
                    if ((ele[2]==n[1]) and (ele[3]==n[2])):
                        q_index = i
                        
                if q_index != -1:
                    q[q_index][0] = tot_n
                    q[q_index][1] = n[0]
                    q[q_index][1] = n[3]
            
        else:
            # No we have not encountered it
            
            c += 1    # incrementing counter
            
            ind_mat[n[1]*2,n[2]*2,int(n[3]/30)] = c
            
            temp_element = [tot_n,n[0],n[1],n[2],n[3],c]
            
            temp_graph_element = [n[1],n[2],n[3],n[0],point[5]]
            
            graph = np.vstack((graph,temp_graph_element)) # vertically stacking the new node to the graph
            
            hq.heappush(q,temp_element)     # adding the element to the heapq
    
    visited = np.vstack((visited,[point[2],point[3]]))

# End of the Loop

end_time = datetime.now()
time_taken = end_time - start_time      # calculating the time taken to complete the search

# Print the time taken
print("Time taken:", time_taken)


ind = np.where((graph[:, 0] == goal_point[0]) & (graph[:, 1] == goal_point[1]) & (graph[:, 2] == goal_point[2]))[0]

path_x = []
path_y = []
while ind !=-2:
    v = (graph[ind,0],graph[ind,1])
    path_x.append(v[0])
    path_y.append(v[1])
    ind = graph[ind,4]
path_xrev = path_x[::-1]
path_yrev = path_y[::-1]

# storing the path coordinates and the visited list in  .txt files, to visulaize without running the code again

f = open("Path.txt","w")
for i in range(len(path_xrev)):
    f.write(f"{path_xrev[i]} {path_yrev[i]}")
    f.write("\n")

f.close()

g = open("Visited.txt","w")
for i in visited:
    for j in i:
        g.write(f"{j} ")
    g.write("\n")

g.close()

ready = input(" READY FOR THE ANIMATION ( PRESS Y )")

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
    pygame.draw.circle(canvas, red, (start_x, 500 - start_y), 1)
    pygame.draw.circle(canvas, blue, (goal_x, 500 - goal_y), 1)
    pygame.display.flip()

    for i in visited:
        pygame.draw.circle(canvas, white, (int(i[0]), int(500 - i[1])), 1)
        pygame.draw.circle(canvas, red, (start_x, 500 - start_y), 1)
        pygame.display.flip()

    for i in range(len(path_xrev)):
        pygame.draw.circle(canvas, black, (int(path_xrev[i]), int(500 - path_yrev[i])), 1)
        pygame.display.flip()
        pygame.time.delay(10)

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(1)  # Adjust the frame rate as needed

# Quit Pygame
pygame.quit()
sys.exit()
