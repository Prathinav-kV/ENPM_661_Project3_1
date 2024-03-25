# ENPM_661_Project3_1
ENPM Project 3 Phase 1

# ENPM-661-project2
ENPM 661 Project 2
Dijkstras algorithm Project 2:

Prathinav, Karnala Venkata
120380983
pratkv

Sarang, Shibu
120254307
sarang

github link: https://github.com/Prathinav-kV/ENPM_661_Project3_1

YouTube link: 

Code execution:

1. When running the code, it will ask you for the clearance distance, the stride length,start and goal coordinates along with their orientation angles.
Input the x coordinate and then input the y coordinate, based on the bottom left origin (0,0). It checks if the inputted coordinates are lying on the object space or not. It will ask you to input the right coordinates in case they fall in the obstacle space. It also ensures that the start node and goal node are not the same.

eg: 
    clearance : 10,
    stride : 4,
    start x coordinate: 20,
    start y coordinate: 20,
    start orientation: 30
    -------------------------
    goal x coordinate: 400,
    goal y coordinate: 450,
    goal orientation: 60

2. We have defined the equations taking into consideration the robot radius =5 units and the clearance as inputted . We calculated the equations of all the boundaries and used them in the valid_point(x,y) function. It checks if the point lies in the obstacle space or not, and returns a boolean value where True means it is a valid point and False means it is in the obstacle space.

3. The graph is a numpy matrix where each element is structured in this manner:
[ x,y,theta,cost to come,parent node index]

The heapq has elements arranged as follows:
[ ctc+ctg, ctc, x, y, theta, node index]

4. We are moving the points in 7 directions using the following angles [30,60,90,0,-30,-60,-90]
We took a goal threshold of 0.5 units.

4. The animation is not real-time, we perform the animation after computing the path.

5. The red dot is the starting index.
The blue dot is the goal index.
The black line is the final path.
The white portion in the animation is the visited list. It does not grow from the starting point, it just starts all over.

6. There are 2 text files. Visted.txt is the list containing all the visited nodes. Path.txt contains the coordinates of the path. In the files, i have the path for the example case as mentioned in step 1.

Libraries used:

1.pygame
2.heapq 
3.numpy
4.datetime
5.sys 


