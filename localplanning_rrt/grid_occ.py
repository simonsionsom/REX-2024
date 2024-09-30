"""
Module for interfacing a 2D Map in the form of Grid Occupancy
"""

import numpy as np
import matplotlib.pyplot as plt

class GridOccupancyMap(object):
    def __init__(self, low=(0, 0), high=(2, 2), res=0.05) -> None:
        self.map_area = [low, high]    # A rectangular area    
        self.map_size = np.array([high[0] - low[0], high[1] - low[1]])
        self.resolution = res

        # Number of grid cells in each direction
        self.n_grids = [int(s // res) for s in self.map_size]

        # Create an empty grid
        self.grid = np.zeros((self.n_grids[0], self.n_grids[1]), dtype=np.uint8)

        # Extent for plotting
        self.extent = [self.map_area[0][0], self.map_area[1][0], self.map_area[0][1], self.map_area[1][1]]

    def draw_grid(self):
        """
        Draw the grid system with coordinates
        """
        fig, ax = plt.subplots()

        # Draw vertical lines (x-grid)
        for i in range(self.n_grids[0] + 1):
            x = self.map_area[0][0] + i * self.resolution
            ax.plot([x, x], [self.map_area[0][1], self.map_area[1][1]], color='black', linewidth=0.5)

        # Draw horizontal lines (y-grid)
        for j in range(self.n_grids[1] + 1):
            y = self.map_area[0][1] + j * self.resolution
            ax.plot([self.map_area[0][0], self.map_area[1][0]], [y, y], color='black', linewidth=0.5)

        # Add labels to each grid cell
        for i in range(self.n_grids[0]):
            for j in range(self.n_grids[1]):
                x_center = self.map_area[0][0] + i * self.resolution + self.resolution / 2
                y_center = self.map_area[0][1] + j * self.resolution + self.resolution / 2
                ax.text(x_center, y_center, f"({i}, {j})", ha='center', va='center', fontsize=6)

        # Set the aspect ratio to 'equal' so the grid isn't distorted
        ax.set_aspect('equal')

        # Set the axis limits and labels
        ax.set_xlim(self.map_area[0][0], self.map_area[1][0])
        ax.set_ylim(self.map_area[0][1], self.map_area[1][1])
        ax.set_xlabel("X-coordinate")
        ax.set_ylabel("Y-coordinate")

        # Show the grid
        plt.show()

if __name__ == '__main__':
    map = GridOccupancyMap()
    
    plt.clf()
    map.draw_grid()
     