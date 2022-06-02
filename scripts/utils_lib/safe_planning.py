import numpy as np
from scipy import ndimage

def wrap_angle(angle):
    # Wrap angle function to keep an angle betwee pi and -pi
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class PathValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose, scale = 1): 
        
        p = self.__position_to_map__(np.asarray((pose[0], pose[1]))) # : convert world robot position to map coordinates using method __position_to_map__

        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribude). 
        # Be aware to only generate elements inside the map.
        vicinity = 3 #int(self.distance/self.resolution/scale)
        x_width = self.map.shape[0]; y_length = self.map.shape[1]
          
        if len(p) == 2: # if p is outside the map return true (unexplored positions are considered free)
            u_min = p[0] - vicinity if p[0] - vicinity  > 0 else 0 # x min
            v_min = p[1] - vicinity if p[1] - vicinity  > 0 else 0 # y min
            u_max = p[0] + vicinity if p[0] + vicinity  < x_width - 1 else x_width - 1 # x max
            v_max = p[1] + vicinity if p[1] + vicinity  < y_length - 1 else y_length - 1 # y max
            if np.any(self.map[u_min:u_max, v_min:v_max] > 0):
                return False                                        # Obstacle
        return True

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path, step_size = 0.1):

        for i in range(len(path) - 1):

            # Get dist and angle between current element and next element in path
            current_p = (path[i,0], path[i,1])
            next_p = (path[i+1,0], path[i+1,1])
            dx = next_p[0] - current_p[0]; dy = next_p[1] - current_p[1]

            angle = np.arctan2(dy, dx)
            dist = np.hypot(dx, dy)

            for d in np.arange(0.0, dist, step_size):
                px = current_p[0] + d*np.cos(angle)
                py = current_p[1] + d*np.sin(angle)
                p = (px, py)
                
                if not self.is_valid(p):
                    return False
        return True

    # Function to inflate obstcles on a gridmap for security purposes in order to plan safe paths away from obstacles
    def inflate_obs(self, map):
        map = np.where(map < 0, 0, map)
        struct2 = ndimage.generate_binary_structure(2, 2)
        inflated_map = ndimage.binary_dilation(map,structure=struct2, iterations=3)
        return 100*inflated_map

    def __position_to_map__(self, p): # P has to be a numpy array

        # Convert world position to map coordinates
        uv = (p - self.origin) / self.resolution

        # Keep position inside map
        if uv[0] < 0 or uv[0] >= self.map.shape[0] or uv[1] < 0 or uv[1] >= self.map.shape[1]:
            return []
        return uv.astype(int)
    
    def __map_to_position__(self, cell):

        # Convert map coordinates to world position

        p = cell* self.resolution + self.origin

        return p