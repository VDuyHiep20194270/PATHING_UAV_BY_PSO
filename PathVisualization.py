import numpy as np 
import matplotlib.pyplot as plt
import numpy.random as rnd
import imageio


# function to plot a path
def show_path(path,path2,map,dim,colormap,start,start2,goal,goal2):
    path = np.array(path)
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    x = np.append(np.array(start[0]),np.append(x,goal[0]))
    y = np.append(np.array(start[1]),np.append(y,goal[1]))
    
    plt.matshow(map,cmap=colormap)
    plt.title('Fastest path')
    plt.plot(y,x)
    path2 = np.array(path2)
    x = [point[0] for point in path2]
    y = [point[1] for point in path2]
    x = np.append(np.array(start2[0]),np.append(x,goal2[0]))
    y = np.append(np.array(start2[1]),np.append(y,goal2[1]))
    plt.plot(y,x)

def show_path2(path,map,dim,colormap,goal):
    path = np.array(path)
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    x = np.append(np.array([0]),np.append(x,goal[0]))
    y = np.append(np.array([0]),np.append(y,goal[1]))
    
    plt.matshow(map,cmap=colormap)
    plt.title('Fastest path')
    plt.plot(y,x)
# function to generate random maps with obstacles
def generate_map(num_obstacles,dim):
    map = np.zeros((dim,dim))

    Y, X = np.ogrid[:dim, :dim]
    dist_from_center = np.sqrt((X - dim/2)**2 + (Y-dim/2)**2)
    mask = (dist_from_center <= 2)
    map[mask] = 255

    for i in range(num_obstacles-1):
        center = rnd.randint(0+(dim/10),dim-(dim/10),size=2)
        radius = rnd.randint(dim/15,dim/10)
        Y, X = np.ogrid[:dim, :dim]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)
        mask = (dist_from_center <= radius)
        map[mask] = 255

    return map

# function to create a gif of the optimization process
def create_gif(iteration):
    images = []
    for i in range(1,iteration):
        images.append(imageio.imread('./images/animation/iteration'+str(i)+'.png'))
    imageio.mimsave('./images/anim.gif', images)