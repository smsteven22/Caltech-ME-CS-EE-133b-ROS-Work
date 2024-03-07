#!/usr/bin/env python3
#
#   carplanner.py
#
#   Plan car movements.
#
import bisect
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import pi, sin, cos, tan, atan2, sqrt, ceil, floor
from shapely.geometry   import Point, MultiPoint, LineString, MultiLineString
from shapely.geometry   import Polygon, MultiPolygon
from shapely.prepared   import prep


######################################################################
# SECTION 1: SCENARIO AND ALGORITHM PARAMETERS
######################################################################
#
#   Algorithm Parameters
#
#   Scenario.  Choose from:
#     'parallel parking'        HW7 Problem 4
#     'u turn'                  HW7 Problem 5
#     'move over'               HW7 Problem 6
#
#   Costs:
#     cstep             Cost per each step
#     csteer            Cost for each change in steering angle
#     creverse          Cost for each change in fworward/back direction
#
#   Grid Sizes:
#     dstep             Distance (meters) per grid cell
#     thetastep         Angle (radian) per grid cell
#
#   Please change the scenario and the costs as you explore
#

# Select the scenario.
scenario = 'parallel parking'
#scenario = 'u turn'
#scenario = 'move over'

# Choose the cost coefficients.
# Default coefficients:
cstep    =   1
csteer   =  10
creverse = 100

"""
Problem 4, Shortest possible path:
cstep = 1
csteer = 0
creverse = 0
"""

"""
Problem 5
Smoothest path:
cstep = 1
csteer = 0
creverse = 100

Pull forward then back up:
cstep = 1
csteer = 0
creverse = 10
"""

"""
Problem 6, Three-point turn
cstep = 1
csteer = 100
creverse = 0
"""

# Choose the grid sizes.
if scenario == 'parallel parking':
    dstep     = 0.4             # Distrance driven per move
    thetastep = pi/36           # Angle rotated per non-straight move

elif scenario == 'u turn':
    dstep     = 0.5             # Distrance driven per move
    thetastep = pi/36           # Angle rotated per non-straight move

elif scenario == 'move over':
    dstep     = 0.5             # Distrance driven per move
    thetastep = pi/36           # Angle rotated per non-straight move

else:
    raise ValueError("Unknown Scenario")


######################################################################
# SECTION 2: CAR/WORLD DEFINITIONS
######################################################################
#
#   Car Definitions
#
#   Define the car dimensions and parameters.
#

# Car outline.
(lcar, wcar) = (4, 2)           # Length/width

# Center of rotation (center of rear axle).
lb        = 0.5                 # Center of rotation to back bumper
lf        = lcar - lb           # Center of rotation to front bumper
wc        = wcar/2              # Center of rotation to left/right
wheelbase = 3                   # Center of rotation to front wheels


######################################################################
#
#   World Defintions
#
#   This should select:
#
#   (xmin, xmax, ymin, ymax)        # Overall dimensions
#   (xstart, ystart, thetastart)    # Starting pose
#   (xgoal,  ygoal,  thetagoal)     # Goal pose
#   walls                           # Shapely MultiLineString object
#

### Parallel Parking:
if scenario == 'parallel parking':

    # General numbers.
    (wroad)                  = 6                # Road width
    (xspace, lspace, wspace) = (3, 6, 2.5)      # Parking Space pos/len/width

    # Overall size.
    (xmin, xmax) = (0, 14)
    (ymin, ymax) = (0, wroad+wspace)

    # Pick your start and goal locations.
    (xstart, ystart, thetastart) = (1.0, 4.0, 0.0)
    (xgoal,  ygoal,  thetagoal)  = (xspace+(lspace-lcar)/2+lb, wroad+wc, 0.0)

    # Define the walls.
    walls = prep(MultiLineString([LineString([
        [xmin, ymin], [xmax, ymin], [xmax, wroad], [xspace+lspace, wroad],
        [xspace+lspace, wroad+wspace], [xspace, wroad+wspace],
        [xspace, wroad], [xmin, wroad], [xmin, ymin]])]))

### U Turn (3 point turn).
elif scenario == 'u turn':

    # General numbers.
    (xroad, yroad, wlane) = (5, 10, 3)  # road len, pos, lane width

    # Overall size.
    (xmin, xmax) = (0, 22)
    (ymin, ymax) = (0, 20)

    # Pick your start and goal locations.
    (xstart, ystart, thetastart) = (1.0, yroad-1.5, 0)
    (xgoal,  ygoal,  thetagoal)  = (4.0, yroad+1.5, pi)

    # Define the walls.
    walls = prep(MultiLineString([LineString([
        [xmin, yroad-wlane], [xroad, yroad-wlane], [xroad, ymin],
        [xmax, ymin], [xmax, ymax], [xroad, ymax], [xroad, yroad+wlane],
        [xmin, yroad+wlane], [xmin, yroad-wlane]])]))

### Move/Shift over
elif scenario == 'move over':

    # General numbers.
    (xrail, yrail, lrail) = (7.5, 4, 5)          # Guard rail pos/len

    # Overall size.
    (xmin, xmax) = (0, 20)
    (ymin, ymax) = (0, 20)

    # Pick your start and goal locations.
    (xstart, ystart, thetastart) = (8.5, 2.0, 0.0)
    (xgoal,  ygoal,  thetagoal)  = (8.5, 6.0, 0.0)

    # Define the walls.
    walls = prep(MultiLineString([
        LineString([[xmin, ymin], [xmax, ymin], [xmax, ymax],
                 [xmin, ymax], [xmin, ymin]]),
        LineString([[xrail, yrail], [xrail+lrail, yrail]])]))

### Unknown.
else:
    raise ValueError("Unknown Scenario")


######################################################################
# SECTION 3: VISUALIZATION
######################################################################
#
#   Visualization Utility Class
#
class Visualization:
    def __init__(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_aspect('equal')

        # Show the walls.
        xticks = set([xstart, xgoal])
        yticks = set([ystart, ygoal])
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
        # Show the box/frame.
        plt.plot(*node.box.exterior.xy, *args, **kwargs)
        # Add headlights set in 10% from front corners.
        (flx,fly) = node.box.exterior.coords[0]
        (frx,fry) = node.box.exterior.coords[3]
        plt.plot(0.9*frx+0.1*flx, 0.9*fry+0.1*fly, *args, **kwargs, marker='o')
        plt.plot(0.1*frx+0.9*flx, 0.1*fry+0.9*fly, *args, **kwargs, marker='o')

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot([head.x, tail.x], [head.y, tail.y], *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for node in path:
            self.drawNode(node, *args, **kwargs)
            self.show(delay = 0.1)



######################################################################
# SECTION 4: NODE
#
#   Node Definition
#
# Initialize the counters (for diagnostics only)
nodecounter = 0
donecounter = 0

# Make sure thetastep is an integer fraction of 2pi, so the grid wraps nicely.
#thetastep = dstep * tan(steerangle) / wheelbase
thetastep  = 2*pi / round(2*pi/thetastep)
steerangle = np.arctan(wheelbase * thetastep / dstep)

# Node Class
class Node:
    def __init__(self, x, y, theta):
        # Setup the basic A* node.
        super().__init__()

        # Remember the state (x,y,theta).
        self.x     = x
        self.y     = y
        self.theta = theta

        # Box = 4 corners: frontleft, backleft, backright, frontright
        (s,c) = (np.sin(theta), np.cos(theta))
        self.box = Polygon([[x + c*lf - s*wc, y + s*lf + c*wc],
                            [x - c*lb - s*wc, y - s*lb + c*wc],
                            [x - c*lb + s*wc, y - s*lb - c*wc],
                            [x + c*lf + s*wc, y + s*lf - c*wc]])

        # Tree connectivity.  Define how we got here (set defaults for now).
        self.parent  = None
        self.forward = 1        # +1 = drove forward, -1 = backward
        self.steer   = 0        # +1 = turned left, 0 = straight, -1 = right

        # Cost/status.
        self.cost = 0           # Cost to get here.
        self.done = False       # The path here is optimal.

        # Count the node - for diagnostics only.
        global nodecounter
        nodecounter += 1
        if nodecounter % 1000 == 0:
            print("Sampled %d nodes... fully processed %d nodes" %
                  (nodecounter, donecounter))

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<XY %5.2f,%5.2f @ %5.1f deg> (fwd %2d, str %2d, cost %4d)" %
                (self.x, self.y, np.degrees(self.theta),
                 self.forward, self.steer, self.cost))

    # Define the "less-than" to enable sorting by cost!
    def __lt__(self, other):
        return (self.cost < other.cost)

    # Determine the grid indices based on the coordinates.  The x/y
    # coordinates are regular numbers, the theta maps to 0..360deg!
    def indices(self):
        return (round((self.x - xmin)/dstep),
                round((self.y - ymin)/dstep),
                round(self.theta/thetastep) % round(2*pi/thetastep))


    #####################
    # Forward Simulations
    # Instantiate a new (child) node, based on driving from this node.
    def nextNode(self, forward, steer):

        # Problem 1, write the simulation (compute the IVP).
        # Assume forward = +1,    -1
        #        steer   = +1, 0, -1
        # and read dstep and thetastep to get magnitudes.
        xnext = self.x + (forward * dstep) * cos(self.theta + (((forward * steer) * thetastep) / 2)) * np.sinc(((forward * steer) * thetastep) / 2)
        ynext = self.y + (forward * dstep) * sin(self.theta + (((forward * steer) * thetastep) / 2)) * np.sinc(((forward * steer) * thetastep) / 2)
        thetanext = self.theta + ((forward * steer) * thetastep)

        # Problem 2, compute a cost to reach each node
        Nstep = 1
        NsteeringAnglesChanges = 0
        Nreversals = 0
        if self.steer != steer:
            NsteeringAnglesChanges = 1

        if self.forward != forward:
            Nreversals = 1

        c = (cstep * Nstep) + (csteer * NsteeringAnglesChanges) + (creverse * Nreversals)

        # Create the child node.
        child = Node(xnext, ynext, thetanext)

        # Set the parent relationship.
        child.parent  = self
        child.forward = forward
        child.steer   = steer
        child.cost = self.cost + c

        # Return
        return child


    ######################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        return walls.disjoint(self.box)

    # Check the local planner - whether this connects to another node.
    # This is slightly approximate, but good for the small steps.
    def connectsTo(self, other):
        return walls.disjoint(self.box.union(other.box).convex_hull)


######################################################################
# SECTION 5: PLANNER
#
#   Planner Functions
#
def planner(startnode, goalnode):
    # Create the grid to store one Node per square.  Add 1 to the x/y
    # dimensions to include min and max values.  See Node.indices().
    grid = np.empty((1 + int((xmax - xmin) / dstep),
                     1 + int((ymax - ymin) / dstep),
                     round(2*pi / thetastep)), dtype='object')
    print("Created %dx%dx%d grid (with %d elements)" %
          (np.shape(grid) + (np.size(grid),)))

    # Prepare the still empty *sorted* on-deck queue.
    onDeck = []

    # Begin with the start node on-deck and in the grid.
    grid[startnode.indices()] = startnode
    bisect.insort(onDeck, startnode)

    # Problem 3
    # Continually expand/build the search tree.
    while True:
        # Make sure we have something pending in the on-deck queue.
        # Otherwise we were unable to find a path!
        if not (len(onDeck) > 0):
            return None

        # Grab the next node (first on deck).
        node = onDeck.pop(0)

        # Mark this node as done.
        node.done = True
        global donecounter
        donecounter += 1

        # Problem 3, Grid-Based Car Planning Algorithm
        # Break the loop if done.

        if node.indices() == goalnode.indices():
            grid[node.indices()] = goalnode
            bisect.insort(onDeck, goalnode)
            break

        forward = [1, -1]
        steer = [1, 0, -1]

        for i in forward:
            for j in steer:
                child = node.nextNode(i, j)
 
                if grid[child.indices()] == node:
                    child = child.nextNode(i, j)
                    
                if child.inFreespace() and node.connectsTo(child):
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

    # Return the path.
    return path

    


######################################################################
# SECTION 6: TESTING AND MAIN FUNCTION
#
#  Test Functions
#
def testdriving(visual):
    # Test with three repeated action.
    def testdrive1(visual, node, forward, steer, color):
        visual.drawNode(node, color='k', linewidth=2)
        for i in range(3):
            node = node.nextNode(forward, steer)
            visual.drawNode(node, color=color, linewidth=2)

    # Try all six actions.
    testdrive1(visual, Node(8.0, 7.0, 0.0),  1.0, -1.0, 'r')
    testdrive1(visual, Node(8.0, 4.0, 0.0),  1.0,  0.0, 'g')
    testdrive1(visual, Node(8.0, 1.0, 0.0),  1.0,  1.0, 'b')
    testdrive1(visual, Node(3.0, 7.0, 0.0), -1.0, -1.0, 'm')
    testdrive1(visual, Node(3.0, 4.0, 0.0), -1.0,  0.0, 'y')
    testdrive1(visual, Node(3.0, 1.0, 0.0), -1.0,  1.0, 'c')
    visual.show("Showing the driving test cases")

def testcosts(visual):
    nodes = [Node(4.0, 4.0, 0.0)]
    nodes.append(nodes[-1].nextNode(1, 0))
    nodes.append(nodes[-1].nextNode(1, 0))
    nodes.append(nodes[-1].nextNode(1, 1))
    nodes.append(nodes[-1].nextNode(1, 1))
    nodes.append(nodes[-1].nextNode(1, -1))
    nodes.append(nodes[-1].nextNode(1, -1))
    nodes.append(nodes[-1].nextNode(-1, 0))
    nodes.append(nodes[-1].nextNode(-1, 0))
    for node in nodes:
        print(node)


######################################################################
#
#   Main Code
#
def main():
    # Create the figure.
    visual = Visualization()

    # Testing!  FIXME: Change to False to run the regular code.
    # if True:
        # testdriving(visual)
        # return
    if False:
        testcosts(visual)
        return

    # Report the parameters.
    print('Running with step size %.2fm and delta-theta of 360/%d = %.2fdeg' %
          (dstep, int(2*pi/thetastep), np.degrees(thetastep)))
    print('So the possible steering angles are 0 or +/- %.2fdeg' %
          (np.degrees(steerangle)))
    print('Cost %d per move plus %d per steering and %d per direction change' %
          (cstep, csteer, creverse))

    # Create the start/goal nodes.
    startnode = Node(xstart, ystart, thetastart)
    goalnode  = Node(xgoal,  ygoal,  thetagoal)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='c', linewidth=2)
    visual.drawNode(goalnode,  color='m', linewidth=2)
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
