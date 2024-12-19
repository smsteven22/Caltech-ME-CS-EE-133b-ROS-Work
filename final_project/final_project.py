import bisect
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import pi, sin, cos, tan, atan2, sqrt, ceil, floor
from shapely.geometry   import Point, MultiPoint, LineString, MultiLineString
from shapely.geometry   import Polygon, MultiPolygon
from shapely.prepared   import prep

"""
Section 1: Algorithm Parameters

Costs:
    cstep           Cost per each step
    chorizontal     Cost for each change in horizontal direction (left or right)
    cvertical       Cost for each change in vertical direction (up or down)

Grid Sizes:
    dstep           Distance (meters) per grid cell
"""

cstep = 1
chorizontal = 100
cvertical = 1

dstep = 1

"""
Section 2: World Definitions

(xmin, xmax, ymin, ymax)    Overall Dimensions
(xstart, ystart)            Starting position
(xgoal, ygoal)              Goal position
walls                       Shapely MultiLineString object
"""
# General Numbers
(lroad, wroad) = (20, 16)  # Field width

(xspace, lspace, wspace) = (8, 4, 2) # Goal pos/len/width

# Overall size
(xmin, xmax) = (0, lroad)
(ymin, ymax) = (0, wroad+wspace)

# Start and goal locations
(xstart_robot, ystart_robot) = (16, 2)
(xstart_ball, ystart_ball) = (5, 6)
(xgoal, ygoal) = (xspace+(lspace/2), wroad+wspace - 1)

# Locations of obstacles (2x2 boxes)
# The x/y coordinate is the center of the box.
(obstx1, obsty1) = (3, 1)
(obstx2, obsty2) = (3, 3)
(obstx3, obsty3) = (10, 3)
(obstx4, obsty4) = (12, 5)
(obstx5, obsty5) = (14, 5)
(obstx6, obsty6) = (16, 7)
(obstx7, obsty7) = (19, 3)
(obstx8, obsty8) = (4, 11)
(obstx9, obsty9) = (6, 9)
(obstx10, obsty10) = (8, 11)
(obstx11, obsty11) = (10, 9)
(obstx12, obsty12) = (15, 13)
(obstx13, obsty13) = (17, 13)
(obstx14, obsty14) = (17, 11)
(obstx15, obsty15) = (4, 15)

# Define the walls.
walls = prep(MultiLineString([
    LineString([[xmin, ymin], [xmax, ymin], [xmax, wroad], 
                [xspace+lspace, wroad],[xspace+lspace, wroad+wspace], 
                [xspace, wroad+wspace],[xspace, wroad], [xmin, wroad], 
                [xmin, ymin]]),
    LineString([[obstx1-1, obsty1-1], [obstx1+1, obsty1-1], [obstx1+1, obsty1+1], 
                [obstx1-1, obsty1+1], [obstx1-1, obsty1-1]]),
    LineString([[obstx2-1, obsty2-1], [obstx2+1, obsty2-1], [obstx2+1, obsty2+1], 
                [obstx2-1, obsty2+1], [obstx2-1, obsty2-1]]),
    LineString([[obstx3-1, obsty3-1], [obstx3+1, obsty3-1], [obstx3+1, obsty3+1], 
                [obstx3-1, obsty3+1], [obstx3-1, obsty3-1]]),
    LineString([[obstx4-1, obsty4-1], [obstx4+1, obsty4-1], [obstx4+1, obsty4+1], 
                [obstx4-1, obsty4+1], [obstx4-1, obsty4-1]]),
    LineString([[obstx5-1, obsty5-1], [obstx5+1, obsty5-1], [obstx5+1, obsty5+1], 
                [obstx5-1, obsty5+1], [obstx5-1, obsty5-1]]),
    LineString([[obstx6-1, obsty6-1], [obstx6+1, obsty6-1], [obstx6+1, obsty6+1], 
                [obstx6-1, obsty6+1], [obstx6-1, obsty6-1]]),
    LineString([[obstx7-1, obsty7-1], [obstx7+1, obsty7-1], [obstx7+1, obsty7+1], 
                [obstx7-1, obsty7+1], [obstx7-1, obsty7-1]]),
    LineString([[obstx8-1, obsty8-1], [obstx8+1, obsty8-1], [obstx8+1, obsty8+1], 
                [obstx8-1, obsty8+1], [obstx8-1, obsty8-1]]),
    LineString([[obstx9-1, obsty9-1], [obstx9+1, obsty9-1], [obstx9+1, obsty9+1], 
                [obstx9-1, obsty9+1], [obstx9-1, obsty9-1]]),
    LineString([[obstx10-1, obsty10-1], [obstx10+1, obsty10-1], [obstx10+1, obsty10+1], 
                [obstx10-1, obsty10+1], [obstx10-1, obsty10-1]]),
    LineString([[obstx11-1, obsty11-1], [obstx11+1, obsty11-1], [obstx11+1, obsty11+1], 
                [obstx11-1, obsty11+1], [obstx11-1, obsty11-1]]),
    LineString([[obstx12-1, obsty12-1], [obstx12+1, obsty12-1], [obstx12+1, obsty12+1], 
                [obstx12-1, obsty12+1], [obstx12-1, obsty12-1]]),
    LineString([[obstx13-1, obsty13-1], [obstx13+1, obsty13-1], [obstx13+1, obsty13+1], 
                [obstx13-1, obsty13+1], [obstx13-1, obsty13-1]]),
    LineString([[obstx14-1, obsty14-1], [obstx14+1, obsty14-1], [obstx14+1, obsty14+1], 
                [obstx14-1, obsty14+1], [obstx14-1, obsty14-1]]),
    LineString([[obstx15-1, obsty15-1], [obstx15+1, obsty15-1], [obstx15+1, obsty15+1], 
                [obstx15-1, obsty15+1], [obstx15-1, obsty15-1]]),]))


"""
Section 3: Visualization
"""
# Visualization Utility Class
class Visualization:
    def __init__(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create new acex, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_aspect('equal')

        # Show the walls
        xticks = set([xstart_robot, xgoal])
        yticks = set([ystart_robot, ygoal])
        for lines in walls.context.geoms:
            xticks.update(lines.xy[0])
            yticks.update(lines.xy[1])
            plt.plot(lines.xy[0], lines.xy[1], 'k-', linewidth=2)
        plt.gca().set_xticks(list(xticks))
        plt.gca().set_yticks(list(yticks))

        # Show.
        self.show()

    def show(self, text = '', delay = 0.001):
        # Show the plot.
        plt.pause(max(0.001, delay))
        # If text is specified, print and maybe wait for confirmation.
        if len(text)>0:
            if delay>0:
                input(text + ' (hit return to continue)')
            else:
                print(text)

    def drawNode(self, node, *args, **kwargs):
        # Show the circle.
        plt.plot(*node.robot.exterior.xy, *args, **kwargs)
        plt.plot(*node.ball.exterior.xy, *args, **kwargs)

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot([head.x, tail.x], [head.y, tail.y], *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for node in path:
            self.drawNode(node, *args, **kwargs)
            self.show(delay = 0.1)

"""
Section 4: Node

Node Definition
"""
# Initialize the counters (for diagnostics only)
nodecounter = 0
donecounter = 0

# Node Class
class Node:
    def __init__(self, robot, ball):
        # Setup the basic A* node.
        super().__init__()

        # Remember the robot state (x, y, r).
        self.robotx = robot[0]
        self.roboty = robot[1]
        self.robotr = robot[2]

        # Remember the ball state (x, y, r).
        self.ballx = ball[0]
        self.bally = ball[1]
        self.ballr = ball[2]

        # Circle
        self.robot = Point(self.robotx, self.roboty).buffer(self.robotr)
        self.ball = Point(self.ballx, self.bally).buffer(self.ballr)

        # Tree connectivity. Define how we got here (set defaults for now).
        self.parent = None
        self.vertical = 1       # +1 = up, 0 = horizontal, -1 = down
        self.horizontal = 0     # +1 = right, 0 = vertical, -1 = left

        # Cost/status.
        self.cost = 0           # The cost to get here.          
        self.done = False       # The path here is optimal.

        # Count the node - for diagnostics only.
        global nodecounter
        nodecounter += 1
        if nodecounter % 1000 == 0:
            print("Sampled %d nodes... fully processed %d nodes" %
                  (nodecounter, donecounter))
        
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Robot XY %5.2f, %5.2f, Ball XY %5.2f, %5.2f> (vert %2d, hori %2d, cost %4d)" %
                (self.robotx, self.roboty, self.ballx, self.bally, self.vertical, self.horizontal, self.cost))
    
    # Define the "less-than" to enable sorting by cost!
    def __lt__(self, other):
        return (self.cost < other.cost)
    
    # Determine the grid indices based on the coordinates.
    # The x/y coordinates are regular numbers.
    def indices(self):
        return(round((self.robotx - xmin)/dstep),
               round((self.roboty - ymin)/dstep),
               round((self.ballx - xmin)/dstep),
               round((self.bally - ymin)/dstep))
    
    # Forward Simulations
    # Instantiate a new (child) node, based on driving from this node.
    def nextNode(self, vertical, horizontal):
        robotxnext = self.robotx + (horizontal * dstep)
        robotynext = self.roboty + (vertical * dstep)

        ballxnext = self.ballx
        ballynext = self.bally

        if (robotxnext == self.ballx and robotynext == self.bally):
            ballxnext = robotxnext + (horizontal * dstep)
            ballynext = robotynext + (vertical * dstep)

        Nstep = 1
        Nvertical = 0
        Nhorizontal = 0
        if self.vertical != vertical:
            Nvertical = 1
        if self.horizontal != horizontal:
            Nhorizontal = 1

        c = (cstep * Nstep) + (cvertical * Nvertical) + (chorizontal * Nhorizontal)

        # Create the child node.
        robotchild = [robotxnext, robotynext, self.robotr]
        ballchild = [ballxnext, ballynext, self.ballr]
        child = Node(robotchild, ballchild)

        # Set the parent relationship.
        child.parent = self
        child.vertical = vertical
        child.horizontal = horizontal
        child.cost = self.cost + c

        # Return
        return child
    
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        return (walls.disjoint(self.robot) and walls.disjoint(self.ball))
    
    # Check the local planner - whether this connects to another node.
    # This is slightly approximate, but good fo the small steps.
    def connectsTo(self, other):
        return (walls.disjoint(self.robot.union(other.robot).convex_hull) and
                walls.disjoint(self.ball.union(other.ball).convex_hull))

"""
Section 5: Planner

Planner Functions
"""
def planner(startnode, goalnode):
    # Create the grid to store one Node per square. Add 1 to the x/y 
    # dimensions to include min and max values. See Node.robotindices() or
    # Node.ballindices()
    grid = np.empty((1 + int((xmax - xmin) / dstep),
                     1 + int((ymax - ymin) / dstep),
                     1 + int((xmax - xmin) / dstep),
                     1 + int((ymax - ymin) / dstep)), 
                     dtype='object')
    print("Created %dx%dx%dx%d grid (with %d elements)" %
          (np.shape(grid) + (np.size(grid),)))
    
    # Prepare the still empty *sorted* on-deck queue.
    onDeck = []

    # Begin with the start node on-deck and in the grid.
    grid[startnode.indices()] = startnode
    bisect.insort(onDeck, startnode)

    # Continually expand/build the search tree.
    while True:
        # Make sure we have something pending in the on-deck queue.
        # Otherwise we are unable to find a path!
        if not (len(onDeck) > 0):
            return None
        
        # Grab the next node (first on deck).
        node = onDeck.pop(0)

        # Mark this node as done.
        node.done = True
        global donecounter
        donecounter += 1

        # Break the loop if done.
        if node.indices() == goalnode.indices():
            grid[node.indices()] = goalnode
            bisect.insort(onDeck, goalnode)
            break
        
        vertical = [-1, 0, 1]
        horizontal = [-1, 0, 1]

        for i in vertical:
            for j in horizontal:
                if i == 0 and j == 0:
                    continue
                
                child = node.nextNode(i, j)
                
                if grid[child.indices()] == node:
                    child = child.nextNode(i, j)

                if child.inFreespace() and child.connectsTo(node):
                    if grid[child.indices()] == None:
                        grid[child.indices()] = child
                        bisect.insort(onDeck, child)
                    else:
                        prev = grid[child.indices()]
                        if prev.done == False and child.cost < prev.cost:
                            onDeck.remove(prev)
                            grid[prev.indices()] = None

                            grid[child.indices()] = child
                            bisect.insort(onDeck, child)

    # Build the path.
    path = [node]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Return the path
    return path

"""
Section 6: Testing and Main Function

Test Functions
"""
def testplanner(visual):
    # Test with three repeated action.
    def testdrive1(visual, node, vertical, horizontal, color):
        visual.drawNode(node, color='k', linewidth=2)
        for i in range(3):
            node = node.nextNode(vertical, horizontal)
            visual.drawNode(node, color=color, linewidth=2)
        
    testdrive1(visual, Node([8.0, 7.0, 0.5], [8.0, 8.0, 0.25]), 1.0, 0.0, 'r')
    visual.show("Showing the planner test cases")



# Main Code
def main():
    # Create the figure.
    visual = Visualization()
    # Testing! FIXME: Change to False to run the regular code.
    if False:
        testplanner(visual)
        return
    

    # Create the start/goal nodes.
    startrobot = [xstart_robot, ystart_robot, 0.5]
    goalrobot = [xgoal, ygoal-1, 0.5]

    startball = [xstart_ball, ystart_ball, 0.25]
    goalball = [xgoal, ygoal, 0.25]

    startnode = Node(startrobot, startball)
    goalnode = Node(goalrobot, goalball)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='c', linewidth=2)
    visual.drawNode(goalnode, color='m', linewidth=2)
    visual.show("Showing basic world", 0)

    # Run the planner.
    print("Running planner...")
    path = planner(startnode, goalnode)

    # If unable to connect, show what we have.
    if not path:
        visual.show("UNABLE TO FIND A PATH")
        return
    
    # Print/Show the path.
    print("PATH found after sampling %d nodes" % nodecounter)
    print("PATH length %d nodes, %d steps" % (len(path), len(path)-1))
    for node in path:
        print(node)
    visual.drawPath(path, color='r', linewidth=2)
    visual.show("Showing the path")

if __name__== "__main__":
    main()