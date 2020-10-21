import rospy
import numpy as np
from json import loads

class GridWorld:
    def __init__(self):
        self.xlims  = rospy.get_param("xlims", [0,20])
        self.ylims  = rospy.get_param("ylims", [0,20])
        
        try:
            self.xlims = loads(self.xlims)
            self.ylims = loads(self.ylims)
        except TypeError:
            pass

        self.res    = rospy.get_param("resolution", 0.4)    # The map resolution [m/cell]
        self.height = rospy.get_param("height", 3)          # Height of search (constant, since 2D estimation)

        x = np.arange(self.xlims[0], self.xlims[1]+self.res, self.res)
        y = np.arange(self.ylims[0], self.ylims[1]+self.res, self.res)

        X,Y = np.meshgrid(x,y)
        
        self.xcell  = X.reshape(X.size)
        self.ycell  = Y.reshape(Y.size)

        # Number of cells in the x and y axes
        self.m      = len(x)
        self.n      = len(y)

        self.M      = self.m * self.n                         # Total number of cells
    
        # Parameters specific to shape of estimate
        self.sx = rospy.get_param("sigma_x", 10)             # Standard deviation (x-axis)
        self.sy = rospy.get_param("sigma_y", 5)              # Standard deviation (y-axis)
        self.mu = rospy.get_param("sensor_accuracy", 0.9)    # Sensor accuracy
        
        # Time between the start of the plume and start of the mission
        self.plume_start = [] # To be edited with rospy time (?)