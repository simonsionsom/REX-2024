import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, pos):
            self.pos = pos  # configuration position, usually 2D/3D for planar robots
            self.path = []  # the path with a integration horizon. straight line for holonomic system
            self.parent = None

        def calc_distance_to(self, to_node):
            d = np.linalg.norm(np.array(to_node.pos) - np.array(self.pos))
            return d

    def __init__(self,
                 start,
                 goal,
                 robot_model,  # model of the robot
                 map,  # map from npy file
                 expand_dis=0.2,
                 path_resolution=0.05,
                 goal_sample_rate=5,
                 max_iter=500,
                 ):

        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.robot = robot_model
        self.map = map

        self.min_rand = map.map_area[0]
        self.max_rand = map.map_area[1]

        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

        self.node_list = []

    def planning(self, animation=True, writer=None):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]

        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision_free(new_node):
                self.node_list.append(new_node)

            if self.node_list[-1].calc_distance_to(self.end) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision_free(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation:
                self.draw_graph(rnd_node)
                if writer is not None:
                    writer.grab_frame()

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.pos)
        d = new_node.calc_distance_to(to_node)

        new_node.path = [new_node.pos]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length // self.path_resolution)

        if n_expand > 0:
            steer_path = self.robot.inverse_dyn(new_node.pos, to_node.pos, n_expand)
            new_node.pos = steer_path[-1]
            new_node.path += steer_path

        d = new_node.calc_distance_to(to_node)
        if d <= self.path_resolution:
            new_node.path.append(to_node.pos)
            new_node.pos = to_node.pos.copy()

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [self.end.pos]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.pos)
            node = node.parent
        path.append(node.pos)

        return path

    def get_random_node(self):
        if np.random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(np.random.uniform(self.map.map_area[0], self.map.map_area[1]))
        else:  # goal point sampling
            rnd = self.Node(self.end.pos)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.pos[0], rnd.pos[1], "^k")

        self.map.draw_map()

        for node in self.node_list:
            if node.parent:
                path = np.array(node.path)
                plt.plot(path[:, 0], path[:, 1], "-g")

        plt.plot(self.start.pos[0], self.start.pos[1], "xr")
        plt.plot(self.end.pos[0], self.end.pos[1], "xr")
        plt.axis(self.map.extent)
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [node.calc_distance_to(rnd_node) for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    def check_collision_free(self, node):
        if node is None:
            return False
        for p in node.path:
            if self.map.in_collision(np.array(p)):
                return False
        return True


class MapFromNpy:
    """
    Map class that loads and processes map from 'mapGod.npy' file
    """

    def __init__(self, map_file):
        self.map_data = np.load(map_file)
        self.map_area = [(0, 0), (self.map_data.shape[1], self.map_data.shape[0])]
        self.extent = [0, self.map_data.shape[1], 0, self.map_data.shape[0]]

    def draw_map(self):
        plt.imshow(self.map_data, cmap='gray', origin='lower', extent=self.extent)

    def in_collision(self, pos):
        x, y = int(pos[0]), int(pos[1])
        return self.map_data[y, x] == 1  # Assuming 1 is obstacle, 0 is free space


import robot_models

def main():
    path_res = 0.05

    # Load the map from the npy file
    map = np.load('Examples/mapGod.npy')

    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])

    rrt = RRT(
        start=[0, 0],  # Starting position
        goal=[0, 1.9],  # Goal position
        robot_model=robot,
        map=map,  # Use the map loaded from the npy file
        expand_dis=0.2,
        path_resolution=path_res,
    )

    show_animation = True
    metadata = dict(title="RRT Test")
    writer = FFMpegWriter(fps=15, metadata=metadata)
    fig = plt.figure()

    with writer.saving(fig, "rrt_test.mp4", 100):
        path = rrt.planning(animation=show_animation, writer=writer)

        if path is None:
            print("Cannot find path")
        else:
            print("Found path!!")

            # Draw final path
            if show_animation:
                rrt.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.01)  # Need for Mac
                plt.show()
                writer.grab_frame()


if __name__ == '__main__':
    main()
