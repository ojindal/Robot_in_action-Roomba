import numpy as np
import matplotlib.pyplot as plt
import math


# define map and its parameters

map_x = 17
map_y = 17

cell_size = 1

no_cell_x = int(map_x // cell_size)
no_cell_y = int(map_y // cell_size)

env_map = np.zeros((no_cell_x, no_cell_y))
obstacle_cmap_value = 25
env_map[0, :] = obstacle_cmap_value
env_map[:, 0] = obstacle_cmap_value
env_map[-1, :] = obstacle_cmap_value
env_map[:, -1] = obstacle_cmap_value

# for i in range(4, 6):
#     for j in range(4, 6):
#         env_map[i, j] = obstacle_cmap_value

# for i in range(8, 12):
#     for j in range(8, 10):
#         env_map[i, j] = obstacle_cmap_value

# motion model!

def motion_model(pos,move):

    moves = {'up': np.array([1, 0]),
           'down': np.array([-1, 0]),
           'left': np.array([0, 1]),
           'right': np.array([0, -1])}

    # given current pose, it return next pos based on control actions if valid
    # if invalid, returns current pose only
    next_pos = pos + moves[move]
    if env_map[next_pos[0],next_pos[1]] == 0:
        valid = True
        return valid, next_pos
    else:
        valid = False
        return valid, pos


# boustrophedon algorithm

start_pos = np.array([1,1])
path = [tuple(start_pos)]
curr  = start_pos
env_map[start_pos[0], start_pos[1]] = 1
counter = 1
count = 0

while 0 in env_map:
    up_valid, next_pos = motion_model(curr,'up')
    #print(curr)
    
    while up_valid:
        #print(curr)
        
        right_valid, next_pos = motion_model(curr,'right')
        
        while right_valid:
            path.append(tuple(next_pos))
            count += 1
            env_map[next_pos[0],next_pos[1]] = counter

            curr = next_pos
            right_valid, next_pos = motion_model(curr,'right')

        up_valid, next_pos = motion_model(curr,'up')

        if up_valid:
            
            path.append(tuple(next_pos))
            count += 1
            env_map[next_pos[0],next_pos[1]] = counter

            curr = next_pos
            up_valid, next_pos = motion_model(curr,'up')

        else:
            up_valid = False

    left_valid, next_pos = motion_model(curr,'left')
    #print(left_valid)
    #print(curr)
    
    if left_valid:
        path.append(tuple(next_pos))
        count += 1
        env_map[next_pos[0],next_pos[1]] = counter
        curr = next_pos

    down_valid, next_pos = motion_model(curr,'down')
    #print(curr)
    #print(down_valid)
    
    while down_valid:
        #print(curr)
        right_valid, next_pos = motion_model(curr,'right')
        
        while right_valid:
            path.append(tuple(next_pos))
            count += 1
            env_map[next_pos[0],next_pos[1]] = counter

            curr = next_pos
            right_valid, next_pos = motion_model(curr,'right')

        down_valid, next_pos = motion_model(curr,'down')

        if down_valid:
            
            path.append(tuple(next_pos))
            count += 1
            env_map[next_pos[0],next_pos[1]] = counter

            curr = next_pos
            up_valid, next_pos = motion_model(curr,'down')

        else:
            up_valid = False


    left_valid, next_pos = motion_model(curr,'left')
    
    if left_valid:
        path.append(tuple(next_pos))
        count += 1
        env_map[next_pos[0],next_pos[1]] = counter
        curr = next_pos


# Calculate theta based on slope and stack waypoints for PID controller!

x_cord = []
y_cord = []
theta_cord = []

x1 = path[0][0]
y1 = path[0][1]
theta_cord.append(0)

waypoints = np.zeros([len(path),3])

#scale = 0.3048*10/17
scale = 1

for i in range(1,len(path)):
    #print(i)
    x_cord.append(path[i][0]*scale)
    y_cord.append(path[i][1]*scale)

    x2 = x_cord[i-1]
    y2 = y_cord[i-1]
    #orientation = math.atan((y2-y1)/(x2-x1))    

    theta_cord.append(0)

    x1,y1 = x2,y2
    waypoints[i-1] = [x_cord[i-1],y_cord[i-1],theta_cord[i-1]]

waypoints[-1] = [path[-1][0]*scale,path[-1][1]*scale,0]

np.savetxt("waypoints_planner.txt", waypoints)

# Visualization tools!

f, ax = plt.subplots()
ax.imshow( env_map.T, interpolation="none", cmap='gray_r', origin='lower', \
            extent=(-0.5, env_map.shape[0]-0.5, -0.5, env_map.shape[1]-0.5) )
ax.axis([-0.5, env_map.shape[0]-0.5, -0.5, env_map.shape[1]-0.5])
ax.set_xlabel('x')
ax.set_ylabel('y')  
hr = ax.plot(start_pos[0], start_pos[1], 'bs')
#ht = ax.plot(targetstart[0], targetstart[1], 'rs')

hp = ax.plot(x_cord, y_cord, linestyle = 'dashed', color = 'red')

f.canvas.flush_events()
plt.show()

