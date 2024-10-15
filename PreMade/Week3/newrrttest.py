import numpy as np
import matplotlib.pyplot as plt

class Map:
    def __init__(self, map_data):
        self.map_data = map_data
        self.map_area = [0, map_data.shape[1], 0, map_data.shape[0]]  # [min_x, max_x, min_y, max_y]

    def in_collision(self, pos):
        x, y = int(pos[0]), int(pos[1])
        if x < 0 or x >= self.map_data.shape[1] or y < 0 or y >= self.map_data.shape[0]:
            return True  # Out of bounds is considered a collision
        return self.map_data[y, x] == 1  # 1 means obstacle in binary map

    def draw_map(self):
        plt.imshow(self.map_data, cmap='gray')

class RRT:
    class Node:
        def __init__(self, pos):
            self.pos = pos
            self.path = []
            self.parent = None
        
        def calc_distance_to(self, to_node):
            return np.linalg.norm(np.array(to_node.pos) - np.array(self.pos))

    def __init__(self, start, goal, map, expand_dis=5.0, path_resolution=1.0, goal_sample_rate=5, max_iter=500):
        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.map = map
        
        self.min_rand = map.map_area[0]
        self.max_rand = map.map_area[1]

        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

        self.node_list = []

    def planning(self, animation=True):
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

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.pos)
        d = new_node.calc_distance_to(to_node)

        new_node.path = [new_node.pos]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length // self.path_resolution)
        if n_expand > 0:
            for _ in range(n_expand):
                direction = np.array(to_node.pos) - np.array(from_node.pos)
                new_pos = np.array(from_node.pos) + (direction / np.linalg.norm(direction)) * self.path_resolution
                new_node.path.append(new_pos)
                new_node.pos = new_pos

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [self.end.pos]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.pos)
            node = node.parent
        path.append(self.start.pos)

        return path

    def get_random_node(self):
        if np.random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(np.random.uniform(self.map.map_area[0:2], self.map.map_area[2:4]))
        else:
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
        plt.grid(True)
        plt.pause(0.01)

    def check_collision_free(self, node):
        if node is None:
            return False
        for p in node.path:
            if self.map.in_collision(np.array(p)):
                return False
        return True

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [node.calc_distance_to(rnd_node) for node in node_list]
        return dlist.index(min(dlist))

def main():
    # Load map from 'map.npy'
    map_data = np.load('map.npy')
    map_instance = Map(map_data)

    # Define the start and goal positions
    start = [10, 10]
    goal = [map_data.shape[1] - 10, map_data.shape[0] - 10]

    # Create an RRT planner instance
    rrt = RRT(start=start, goal=goal, map=map_instance, expand_dis=5.0, path_resolution=1.0, goal_sample_rate=5, max_iter=500)

    # Run the RRT planning
    path = rrt.planning(animation=True)

    # Plot the final path
    if path is not None:
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], '-r')  # Plot the found path in red
    else:
        print("No path found.")

    # Show the final result
    plt.show()

if __name__ == '__main__':
    main()
