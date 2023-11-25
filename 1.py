import numpy as np
import matplotlib.pyplot as plt 
import random 


#treeNode class
class treeNode:
    def __init__(self, X, Y):
        self.LocationX=X
        self.LocationY=Y
        self.children = []
        self.parent = None

#RRT Algorithm claaa 

class RRTAlgoritm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree= treeNode(start[0], start[1])
        self.goal= treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(numIterations , 5001)
        self.grid = grid 
        self.rho = stepSize
        self.path_distance = 0 
        self.nearestDist = 10000
        self.numWaypoints = 0 
        self.Waypoints = [] 

    def addchild(self, LocationX,LocationY):
        if (LocationX == self.goal.LocationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(LocationX,LocationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
            
        


    def sampleApoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point

    def steerToPoint(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        offset = self.rho * u_hat

        new_location = np.array([locationStart.LocationX + offset[0], locationStart.LocationY + offset[1]])

        if new_location[0] >= self.grid.shape[1]:
            new_location[0] = self.grid.shape[1]
        if new_location[1] >= self.grid.shape[0]:
            new_location[1] = self.grid.shape[0]

        return new_location

    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        
        for i in range(self.rho):
            testPoint[0] = locationStart.LocationX + i*u_hat[0]
            testPoint[1] = locationStart.LocationY + i*u_hat[1]
        
            x_idx= int(round(testPoint[0]))
            y_idx= int(round(testPoint[1]))

            if 0 <= x_idx < self.grid.shape[1] and 0 <= y_idx < self.grid.shape[0]:
                if np.any(self.grid[y_idx, x_idx] ==1):
                    return True
        
        return False


    def unitVector(self, locationStart, locationEnd):
        v= np.array([locationEnd[0] - locationStart.LocationX, locationEnd[1] - locationStart.LocationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist

        for child in root.children:
            self.findNearest(child, point)
        pass

    def distance(self, node1, point):
        dist = np.sqrt((node1.LocationX - point[0])**2 + (node1.LocationY - point[1])**2)
        return dist 

    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True 
        pass

    def resetNearestValue(self):
        self.nearestNode = None 
        self.nearestDist = 10000

    def retraceRRTPath(self, goal):
        if goal.LocationX == self.randomTree.LocationX:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.LocationX, goal.LocationY])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)



grid=np.load('.\ERC_Project\obstacle3.npy')   
start = np.array([100.0, 100.0])
goal = np.array([850.0, 1100.0])
fig = plt.figure("RRT Algorithm")
startRegion= plt.Circle((start[0], start[1]), 50, color='r' , fill = False)
goalRegion= plt.Circle((goal[0], goal[1]), 50, color='b' , fill = False)



numIterations = 5000
stepSize = 50 

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0],goal[1],'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
ax.add_patch(startRegion)
plt.xlabel("X-axis $(m)$")
plt.ylabel("Y-axis $(m)$")

rrt = RRTAlgoritm(start, goal, numIterations, grid, stepSize)

for i in range(rrt.iterations):
    rrt.resetNearestValue()
    print("Iteration:",i)

    point = rrt.sampleApoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if (bool == False):
        rrt.addchild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.LocationX, new[0]], [rrt.nearestNode.LocationY, new[1]], 'go', linestyle="--")
    if (rrt.goalFound(new)):
        rrt.addchild(goal[0],goal[1])
        print("Goal found!")
        break

rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0,start)
print("Number of waypoints: ", rrt.numWaypoints)
print("Path Distance (m): ", rrt.path_distance)
print("waypoints: ", rrt.Waypoints)

for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], "ro", linestyle="--")

    plt.pause(0.10)

plt.show(block=True)





