from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation 
import math
import time

animation = True
# initializing a figure in which the graph will be plotted
fig = plt.figure() 

ox, oy = [], []
#nx, ny is the list of vertexs to be printed in animation
nx, ny = [], []
x,y = [],[]
rx,ry = [],[]
# set obstacle positions in the lists ox,oy
def obstacles():

    for i in range(0, 101):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 91):
        ox.append(100.0)
        oy.append(i)
    for i in range(0, 101): 
        ox.append(i)
        oy.append(90.0)
    for i in range(0, 91):
        ox.append(0)
        oy.append(i)
    for i in range(0, 80):
        ox.append(30.0)
        oy.append(i)
    for i in range(0, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(60,90):
        ox.append(i)
        oy.append(60)

# marking the x-axis and y-axis
axis = plt.axes(xlim =(-10, 110), 
                ylim =(-10, 110)) 
  
# initializing the plotting vairables and setting the attributes for plotting,vertexes contains the data from the algorithm , i.e which vertexs were considered for calculating optimum path
vertexs, = axis.plot([], [],'xy') 
boundary, = axis.plot([], [], '.k')  
final_path, = axis.plot([],[],'-r',lw=1) 
#function that returns initalized variables for animation function
def init(): 
    boundary.set_data([], [])
    vertexs.set_data([],[])
    final_path.set_data([],[])
    return boundary,vertexs,final_path

#function that returns animation frame by frame
def animate(i):
    if i >= len(nx):
        final_path.set_data(rx,ry)
        return boundary,vertexs,final_path

    x.append(nx[i]) 
    y.append(ny[i])
    boundary.set_data(ox,oy)
    vertexs.set_data(x,y)
    return boundary,vertexs


"""
This functions contains the algorithm for calculating optimum path
Input Parameters
ox, oy : x, y coordinates of obstacles(the diameter of obstacles in considered to be 1 unit)
grid_size : the size of the grid, in this case gridsize is 2 as the diameter of the robot is 2
robot_radius: radius of the robot
start_x, start_y : x, y coordinates of the initial position of the robot
goal_x, goal_y : x, y coordinates of the destination

Return Variables
rx,ry : final list of coordinations which lead to optimum path

Note:
The motion Model in the function need to be changed according to the possible motions of the robot 
"""
def Dijkstra(ox,oy,grid_size,robot_radius,start_x,start_y,goal_x,goal_y):
    
    min_x = round(min(ox))
    min_y = round(min(oy))
    max_x = round(max(ox))
    max_y = round(max(oy))
    x_width = round((max_x - min_x) / grid_size)
    y_width = round((max_y - min_y) / grid_size)
    #obstacle grid map , vertex is True if obstacle is present in vicinity and robot cannot move in the direction of obstacle
    obstacle_map = None
    #motion is a list that contains dx,dy,cost , thus this should be changed for different robots
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]
   
    #The index parameter depends on the gridsize and the function returns the absolute position
    def position(index, minp):
        pos = index * grid_size + minp
        return pos

    #distance from goal parameter calculation: manhattan, diagonal, eucledian
    def dist_goal(x,y):
        dx = abs(position(x,min_x) - goal_x)
        dy = abs(position(y,min_y) - goal_y)
        return (dx+dy)
        #return (1 * (dx + dy) + (math.sqrt(2) - 2 * 1) * min(dx, dy))
        #return round(math.sqrt ( dx*dx + dy*dy ))

    #obstacle grid map , vertex is True if obstacle is present in vicinity and robot cannot move in the direction of obstacle
    def obstacle_map(ox, oy):

        # obstacle map generation
        nonlocal obstacle_map
        obstacle_map = [[False for _ in range(y_width)]
                             for _ in range(x_width)]
        for ix in range(x_width):
            x = position(ix, min_x)
            for iy in range(y_width):
                y = position(iy, min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= robot_radius:
                        obstacle_map[ix][iy] = True
                        break

    obstacle_map(ox,oy)
    class Vertex:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost #cost to reach the vertex from the starting point
            self.parent_index = parent_index  # index of previous Vertex
            self.dist = dist_goal(x,y)
            self.f = self.dist+self.cost

    #function that returns the grid index with position as input
    def grid_index(position, minp):
        return round((position - minp) / grid_size)

    def unique_index(vertex):
        #return round((vertex.y - min_y) * x_width + (vertex.x - min_x))
        return vertex.y*max(x_width,y_width)+vertex.x

    #checks wether the vertex is a valid vertex
    def verify_vertex(vertex):
        px = position(vertex.x, min_x)
        py = position(vertex.y, min_y)

        if px < min_x:
            return False
        if py < min_y:
            return False
        if px >= max_x:
            return False
        if py >= max_y:
            return False

        if obstacle_map[vertex.x][vertex.y]:
            return False

        return True
    # function to calculate final path when goal is reached
    def final_path(goal_vertex, closed_set):

        rx, ry = [position(goal_vertex.x, min_x)], [
            position(goal_vertex.y, min_y)]
        parent_index = goal_vertex.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(position(n.x, min_x))
            ry.append(position(n.y, min_y))
            parent_index = n.parent_index

        return rx, ry

    #this is where the algorithm starts
    start_vertex = Vertex(grid_index(start_x, min_x),
                           grid_index(start_y, min_y), 0.0, -1)
    goal_vertex = Vertex(grid_index(goal_x, min_x),
                          grid_index(goal_y, min_y), 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[unique_index(start_vertex)] = start_vertex
    while 1:
        
        c_id = min(open_set, key=lambda o: open_set[o].f)
        current = open_set[c_id]
        # show graph
        if animation:  # pragma: no cover
            nx.append(position(current.x, min_x))
            ny.append(position(current.y, min_y))


        if current.x == goal_vertex.x and current.y == goal_vertex.y:
            print("Reached goal")
            goal_vertex.parent_index = current.parent_index
            goal_vertex.cost = current.cost
            break

        # Remove the item from the open set
        del open_set[c_id]

        # Add it to the closed set
        closed_set[c_id] = current

        # expand search grid based on motion model
        for move_x, move_y, move_cost in motion:

            vertex = Vertex(current.x + move_x,
                             current.y + move_y,
                             current.cost + move_cost, c_id)
            n_id = unique_index(vertex)
            if n_id in closed_set:
                continue

            if not verify_vertex(vertex):
                continue

            if n_id not in open_set:
                open_set[n_id] = vertex  # Discover a new vertex
            else:
                if open_set[n_id].cost >= vertex.cost:
                    # This path is the best until now. record it!
                    open_set[n_id] = vertex

    rx, ry = final_path(goal_vertex, closed_set)
    return rx, ry,

def show_animation():
        anim = FuncAnimation(fig, animate, init_func = init,
                             frames = len(nx)+200, interval = 20, blit = True)
        anim.save('Astar.mp4', writer = 'ffmpeg', fps = 100)

def main():
    obstacles()
    start_x = 1
    start_y = 1
    goal_x = 80
    goal_y = 10 
    grid_size = 2.0 
    robot_radius = 1.0  
    global rx,ry
    rx,ry = Dijkstra(ox, oy, grid_size, robot_radius,start_x, start_y, goal_x, goal_y)

    if animation:
        plt.plot(goal_x,goal_y,'or')
        plt.plot(start_x,start_y,'og')
        show_animation()


if __name__ == '__main__':
    main()  


