import numpy as np
import heapq as hq
import matplotlib.pyplot as plt
from datetime import datetime
from math import cos, sin, radians

def valid_point(x,y,c):
    equations = hexagon_side_equations(c)
    
    if ( (x >= c) and (x<=(1200-c)) and (y<=(500-c)) and (y>=c) ):
        # print( " Inside box ")
        if not ( (x>(100-c)) and (x<(175+c)) and (y > (100-c))) :
            # print(" Outside rectangle 1")
            if not ( (x>(275-c)) and (x<(350+c)) and (y<(400+c)) ):
                # print(" Outside rectangle 2")
                if not ( ( ( y - equations[0][0] * x - equations[0][1] ) < 0 ) and (x>equations[1][0]) and ( ( y - equations[2][0] * x - equations[2][1]) > 0) 
                        and ( ( y - equations[3][0]*x - equations[3][1]) > 0 ) and ( x < equations[4][0]) and ( ( y - equations[5][0]*x - equations[5][1]) < 0 ) ):
                    # print(" Outside hexagon")
                    if not ( (x>(900-c)) and (x<(1100+c)) and (y<(450+c)) and (y>(50-c)) ):
                        # print(" Outside C shaped object")
                        return True
                    else:
                        if ( (y<(375-c)) and (y>(125+c)) and (x<(1020-c)) ):
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
    
    angles = [30,60,0,-30,-60]
    new_points = []
    for angle in angles:
        new_theta = theta+angle
        while new_theta >= 360:
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
while start_th >= 360:
    start_th -=360
while not (valid_point(start_x,start_y,clear)):
    print("Please try another starting point not in the object area")
    start_x = int(input("Enter the starting coordinate x: "))
    start_y = int(input("Enter the starting coordinate y: "))

    
goal_x = int(input("Enter the goal coordinate x: "))
goal_y = int(input("Enter the goal coordinate y: "))
goal_th = int(input("Enter the goal orientation: "))
while goal_th >= 360:
    goal_th -=360
while not (valid_point(goal_x,goal_y,clear)) or ((goal_x == start_x)and(goal_y==start_y)):
    print("Please try another goal point not equal to start point and not in the object area")
    goal_x = int(input("Enter the goal coordinate x: "))
    goal_y = int(input("Enter the goal coordinate y: "))  


# Defining the variables

visited = np.array([0,0]) # visited list
q = []      # the open queue
c = 0       # counter for the nodes
p = -2      # prev index for the starting node


ind_mat = np.ones((2400,1000,12),dtype=int)
ind_mat *= -1

goal_index = 0  # variable to check the goal index
start_element = [-1,0,start_x,start_y,start_th,0] # starting point added as an element to the heapq in the format 
# 0: total_cost, 1: cost, 2: x, 3: y, 4: theta,  5: node index

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
        goal_index = ind_mat[point[2]*2,point[3]*2,point[4]]
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


# path_x = np.array([graph[goal_index,3]])    # storing the x coordinate of the goal node
# path_y = np.array([graph[goal_index,4]])    # storing the y coordinate of the goal node

# path_ind = int(graph[goal_index,2])         # taking the path ind which notes the prev node of the node in the graph

# while path_ind != -2:               # setting the condition where the prev node index reaches that of the starting node
    
#     x = graph[path_ind,3]
#     y = graph[path_ind,4]
    
#     path_x = np.append(path_x,x)
#     path_y = np.append(path_y,y)
    
#     path_ind = int(graph[path_ind,2])
    
# # Reversing the path coordinates
# path_xrev = path_x[::-1]
# path_yrev = path_y[::-1]

# # storing the path coordinates and the visited list in  .txt files, to visulaize without running the code again

# f = open("Path.txt","w")
# for i in range(len(path_xrev)):
#     f.write(f"{path_xrev[i]} {path_yrev[i]}")L
#     f.write("\n")

# f.close()

# g = open("Visted.txt","w")
# for i in visited:
#     for j in i:
#         g.write(f"{j} ")
#     g.write("\n")

# g.close()
