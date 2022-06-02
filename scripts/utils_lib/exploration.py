#!/usr/bin/env python3

from matplotlib.pyplot import contour
import numpy as np
import frontier_detector as fd



class frontier_detection:

    def __init__(self, laser_angle_offset = 0):

        self.frontiers = np.empty((1, 2))

        self.origin1 = None

        self.flag = True

        self.laser_angle_offset = laser_angle_offset

    def detect_frontiers(self, lr, pose, origin, resolution, grid_map):             # robot and origin have to be numpy arrays

        ranges = lr[:,0] # lidar ranges
        angles = lr[:,1] # lidar angles

        points = pol2cart(ranges, angles, pose, self.laser_angle_offset)            # points in cartesian space

        cells = __position_to_cell__(points, origin, resolution)                    # Storing the sensor readings

        # Contour generation and frontier detection
        new_frontiers = np.asarray(fd.FFD(cells, grid_map))                         # Obtaining new frontiers
        
        if new_frontiers.shape[0] > 0:

            # Frontier Filtering
            new_frontiers = __cell_to_position__(new_frontiers, origin, resolution) # Converting frontiers to cartesian positions

            self.frontiers = np.vstack((self.frontiers, new_frontiers))             # staking the new frontiers with the old frontiers

            self.frontiers = np.unique(self.frontiers, axis=0)                      # Removing dupicate frontiers

            # Frontier Maintenance
            checker = __position_to_cell__(self.frontiers, origin, resolution)      # Converting frontiers again to cell positions to remove 
                                                                                    # unnecesary frontier cells
            indexes = []
            gap = 2
            for index in range(len(checker)):
                if np.any(grid_map[checker[index,0].astype(int) - gap: checker[index,0].astype(int) + gap,checker[index,1].astype(int) - gap: checker[index,1].astype(int) + gap] < 0) and not np.any(grid_map[checker[index,0].astype(int) - gap: checker[index,0].astype(int) + gap,checker[index,1].astype(int) - gap: checker[index,1].astype(int) + gap] > 50):
                    continue
                else:
                    indexes.append(index)
                        
            self.frontiers = np.delete(self.frontiers, indexes, axis = 0)   # Delete frontiers that do not exist any more

        return self.frontiers, __cell_to_position__(cells, origin, resolution)

def pol2cart(rho, phi, pose, laser_angle_offset):
    # Polar to cartesian convertor

    x_r = pose[0]
    y_r = pose[1]
    theta_r = pose[2]

    x = x_r + rho * np.cos(phi + theta_r + laser_angle_offset)
    y = y_r + rho * np.sin(phi + theta_r + laser_angle_offset)

    points = np.append(x.reshape(-1,1), y.reshape(-1,1), axis = 1)

    return points

def __position_to_cell__(p, origin, resolution): 
    # Convert world position to map coordinates

    uv = (p - origin) / resolution

    return uv.astype(int)

def __cell_to_position__(cell, origin, resolution):
    # Convert map coordinates to world position

    p = cell* resolution + origin

    return p.astype(float)


