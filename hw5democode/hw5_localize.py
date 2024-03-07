#!/usr/bin/env python3
#
#   hw5_localize.py
#
#   Homework 5 code framework to localize a robot in a grid...
#
#   Places to edit are marked as FIXME.
#
import numpy as np

from hw5_utilities import Visualization, Robot


#
#  Define the Walls
#
w = ['xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x               x               x               x',
     'x                x             x                x',
     'x                 x           x                 x',
     'x        xxxx      x         x                  x',
     'x        x   x      x       x                   x',
     'x        x    x      x     x      xxxxx         x',
     'x        x     x      x   x     xxx   xxx       x',
     'x        x      x      x x     xx       xx      x',
     'x        x       x      x      x         x      x',
     'x        x        x           xx         xx     x',
     'x        x        x           x           x     x',
     'x        x        x           x           x     x',
     'x        x        x           x           x     x',
     'x                 xx         xx           x     x',
     'x                  x         x                  x',
     'x                  xx       xx                  x',
     'x                   xxx   xxx                   x',
     'x                     xxxxx         x           x',
     'x                                   x          xx',
     'x                                   x         xxx',
     'x            x                      x        xxxx',
     'x           xxx                     x       xxxxx',
     'x          xxxxx                    x      xxxxxx',
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx']

walls = np.array([[1.0*(c == 'x') for c in s] for s in w])
rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)


#
#  Prediction
#
#    bel         Grid of probabilities (current belief)
#    drow, dcol  Delta in row/col
#    probCmd     Modeled probability of command executing
#    prd         Grid of probabilities (prediction)
#
def computePrediction(bel, drow, dcol, probCmd = 1):
    # Prepare an empty prediction grid.
    prd = np.zeros((rows,cols)) + 0.001

    # Determine the new probablities (remember to consider walls).
    for i in range(1, rows-1):
        for j in range(1, cols-1):
            if not walls[i+drow][j+dcol]:
                prd[i+drow][j+dcol] += bel[i][j] # Problem 1
                # prd[i+drow][j+dcol] += bel[i][j] * probCmd # Problems 3 and 4
                # prd[i][j] += bel[i][j] * (1 - probCmd) # Problems 3 and 4
            else:
                prd[i][j] += bel[i][j]
                
    # Return the prediction grid
    # prd /= np.sum(np.sum(prd)) # Problem 4
    return prd


#
#  Measurement Update (Correction)
#
#    prior       Grid of prior probabilities (belief)
#    probSensor  Grid of probability that (sensor==True)
#    sensor      Value of sensor
#    post        Grid of posterior probabilities (updated belief)
#
def updateBelief(prior, probSensor, sensor):
    # Create the posterior belief.
    if sensor == True:
        post = probSensor * prior
    else:
        post = (1 - probSensor) * prior

    # Normalize.
    s = np.sum(post)
    if (s == 0.0):
        print("LOST ALL BELIEF - THIS SHOULD NOT HAPPEN!!!!")
    else:
        post = (1.0/s) * post
    return post


#
#  Pre-compute the Sensor Probability Grid
#
#    drow, dcol    Direction in row/col
#    probProximal  List of probability that sensor triggers at dist=(index+1)
#    prob          Grid of probability that (sensor==True)
#
def precomputeSensorProbability(drow, dcol, probProximal = [1.0]):
    # Prepare an empty probability grid.
    prob = np.zeros((rows, cols))

    # Pre-compute the sensor probability on the grid.
    for i in range(1, rows-1):
        for j in range(1, cols-1): 
            """
            Problem 1
            """
            if walls[i+drow][j+dcol]:
                prob[i][j] = 1
            else:
                prob[i][j] = 0

            """
            Problem 2
            for k in range(len(probProximal)):
                if walls[i+(drow * (k+1))][j+(dcol * (k+1))]:
                    prob[i][j] = probProximal[k]
                    break
            """
            
    # Return the computed grid.
    return prob


# 
#
#  Main Code
#
def main():
    # Initialize the robot simulation.
    # Prob 1(a)  robot=Robot(walls)
    # Prob 1(b)  robot=Robot(walls, row=12, col=26)
    # Prob 2     robot=Robot(walls, row=12, col=26, probProximal=[0.9,0.6,0.3])
    # Prob 3     robot=Robot(walls, row=15, col=47, probProximal=[0.9,0.6,0.3], probCmd=0.8)
    # Prob 4     
    robot=Robot(walls, row=15, col=47, probProximal=[0.9,0.6,0.3],probCmd=0.8, kidnap=True)
    # Or to play robot=Robot(walls, probProximal=[0.9,0.6,0.3], probCmd=0.8)


    # Initialize your localization parameters.
    # probCmd = 1.0 # Problem 1 and 2
    # probProximal = [1.0] # Problem 1
    probCmd      = 0.8 # Problems 3 and 4
    probProximal = [0.9,0.6,0.3] # Problems 2, 3, and 4

    # Report.
    print("Localization is assuming probProximal = " + str(probProximal) +
          ", probCmd = " + str(probCmd))


    # Initialize the figure.
    visual = Visualization(walls, robot)

    # Pre-compute the probability grids for each sensor reading.
    probUp    = precomputeSensorProbability(-1,  0, probProximal)
    probRight = precomputeSensorProbability( 0,  1, probProximal)
    probDown  = precomputeSensorProbability( 1,  0, probProximal)
    probLeft  = precomputeSensorProbability( 0, -1, probProximal)

    # Show the sensor probability maps.
    visual.Show(probUp)
    input("Probability of proximal sensor up reporting True")
    visual.Show(probRight)
    input("Probability of proximal sensor right reporting True")
    visual.Show(probDown)
    input("Probability of proximal sensor down reporting True")
    visual.Show(probLeft)
    input("Probability of proximal sensor left reporting True")


    # Start with a uniform belief grid.
    bel = (1.0 - walls) / np.sum(1.0 - walls)


    # Loop continually.
    while True:
        # Show the current belief.  Also show the actual position.
        visual.Show(bel, markRobot=True)

        # Get the command key to determine the direction.
        while True:
            key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
            if   (key == 'q'):  return
            elif (key == 'i'):  (drow, dcol) = (-1,  0) ; break
            elif (key == 'm'):  (drow, dcol) = ( 1,  0) ; break
            elif (key == 'j'):  (drow, dcol) = ( 0, -1) ; break
            elif (key == 'k'):  (drow, dcol) = ( 0,  1) ; break

        # Move the robot in the simulation.
        robot.Command(drow, dcol)


        # Compute a prediction.
        prd = computePrediction(bel, drow, dcol, probCmd)
        #visual.Show(prd)
        #input("Showing the prediction")

        # Check the prediction.
        if abs(np.sum(prd) - 1.0) > 1e-12:
            print("WARNING: Prediction does not add up to 100%")


        # Correct the prediction/execute the measurement update.
        bel = prd
        bel = updateBelief(bel, probUp,    robot.Sensor(-1,  0))
        bel = updateBelief(bel, probRight, robot.Sensor( 0,  1))
        bel = updateBelief(bel, probDown,  robot.Sensor( 1,  0))
        bel = updateBelief(bel, probLeft,  robot.Sensor( 0, -1))


if __name__== "__main__":
    main()
