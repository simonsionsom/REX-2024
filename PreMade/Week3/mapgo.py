"""
Module for interfacing a 2D Map in the form of Grid Occupancy
"""

import numpy as np
import matplotlib.pyplot as plt

class GridOccupancyMap(object):
    """
    """

    low = (0, 0)
    high = (2, 2)
    res = 0.05

    map_area = [low, high]    # a rectangular area    
    map_size = np.array([high[0] - low[0], high[1] - low[1]])
    resolution = res
    
    n_grids = [int(s // 0.05) for s in map_size]

    grid = np.zeros((n_grids[0], n_grids[1]), dtype=np.uint8)

    extent = [map_area[0][0], map_area[1][0], map_area[0][1], map_area[1][1]]

    @staticmethod
    def in_collision(pos):
        """
        find if the position is occupied or not. return if the queried pos is outside the map
        """
        indices = [int((pos[i] - GridOccupancyMap.map_area[0][i]) // GridOccupancyMap.resolution) for i in range(2)]
        for i, ind in enumerate(indices):
            if ind < 0 or ind >= GridOccupancyMap.n_grids[i]:
                return 1
        
        return GridOccupancyMap.grid[indices[0], indices[1]] 

    @staticmethod
    def populate(n_obs=6):
        """
        generate a grid map with some circle shaped obstacles
        """
        origins = np.random.uniform(
            low=GridOccupancyMap.map_area[0] + GridOccupancyMap.map_size[0] * 0.2, 
            high=GridOccupancyMap.map_area[0] + GridOccupancyMap.map_size[0] * 0.2, 
            size=(n_obs, 2))
        radius = np.random.uniform(low=0.1, high=0.3, size=n_obs)
        # fill the grids by checking if the grid centroid is in any of the circle
        for i in range(GridOccupancyMap.n_grids[0]):
            for j in range(GridOccupancyMap.n_grids[1]):
                centroid = np.array([GridOccupancyMap.map_area[0][0] + GridOccupancyMap.resolution * (i + 0.5), 
                                     GridOccupancyMap.map_area[0][1] + GridOccupancyMap.resolution * (j + 0.5)])
                for o, r in zip(origins, radius):
                    if np.linalg.norm(centroid - o) <= r:
                        GridOccupancyMap.grid[i, j] = 1
                        break
    @staticmethod
    def update_map_with_markers(tvecs):
        """
        Update the occupancy grid map with the positions of detected markers.
        """
        for tvec in tvecs:
            pos = (tvec[0][0], tvec[0][1])  # Extract x, y position from tvec
            indices = [int((pos[i] - GridOccupancyMap.map_area[0][i]) // GridOccupancyMap.resolution) for i in range(2)]
            for i, ind in enumerate(indices):
                if 0 <= ind < GridOccupancyMap.n_grids[i]:
                    GridOccupancyMap.grid[indices[0], indices[1]] = 1  # Mark the grid as occupied

    @staticmethod
    def draw_map():
        # note the x-y axes difference between imshow and plot
        plt.imshow(GridOccupancyMap.grid.T, cmap="Greys", origin='lower', vmin=0, vmax=1, extent=GridOccupancyMap.extent, interpolation='none')
def main():
    map = GridOccupancyMap()

    plt.clf()
    map.draw_map()
    plt.show()

if __name__ == '__main__':
    main()