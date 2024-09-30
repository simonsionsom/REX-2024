# map.py

from camera import Camera  # Step 1: Import the Camera class
import numpy as np
import matplotlib as plt
class GridOccupancyMap:
    def __init__(self):
        # Initialize map parameters
        self.map_area = ...
        self.map_size = ...
        self.n_grids = ...
        self.resolution = ...
        self.grid = np.zeros((self.n_grids[0], self.n_grids[1]))

        # Step 2: Initialize the Camera instance
        self.camera = Camera()

    def update_map_with_camera(self):
        # Step 3: Capture data using the Camera instance
        camera_data = self.camera.capture()

        # Step 4: Use the captured data to update the map
        # Assuming camera_data is a list of (x, y) coordinates
        for x, y in camera_data:
            i = int((x - self.map_area[0][0]) / self.resolution)
            j = int((y - self.map_area[0][1]) / self.resolution)
            if 0 <= i < self.n_grids[0] and 0 <= j < self.n_grids[1]:
                self.grid[i, j] = 1

    def draw_map(self):
        plt.imshow(self.grid.T, cmap="Greys", origin='lower', vmin=0, vmax=1, extent=self.extent, interpolation='none')

if __name__ == '__main__':
    map = GridOccupancyMap()
    map.update_map_with_camera()  # Update the map with camera data
    map.draw_map()