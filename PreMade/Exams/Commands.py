import numpy as np
import matplotlib.pyplot as plt
import marker
import RRT_integrated
import robot
from time import sleep


arlo = robot.Robot()
print("Running robot...")

leftSpeed = 44
rightSpeed = 44
thresholdDistance = 300
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
            self.pos = pos      #configuration position, usually 2D/3D for planar robots  
            self.path = []      #the path with a integration horizon. this could be just a straight line for holonomic system
            self.parent = None
        
        def calc_distance_to(self, to_node):
            # node distance can be nontrivial as some form of cost-to-go function for e.g. underactuated system
            # use euclidean norm for basic holonomic point mass or as heuristics
            d = np.linalg.norm(np.array(to_node.pos) - np.array(self.pos))
            return d
        
    def __init__(self,
                 start,
                 goal,
                 robot_model,   #model of the robot
                 map,           #map should allow the algorithm to query if the path in a node is in collision. note this will ignore the robot geom
                 expand_dis=0.2,
                 path_resolution=0.05,
                 goal_sample_rate=5,
                 max_iter=2000,
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

            #try to steer towards the goal if we are already close enough
            if self.node_list[-1].calc_distance_to(self.end) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision_free(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation:
                self.draw_graph(rnd_node)
                if writer is not None:
                    writer.grab_frame()

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        # integrate the robot dynamics towards the sampled position
        # for holonomic point pass robot, this could be straight forward as a straight line in Euclidean space
        # while need some local optimization to find the dynamically closest path otherwise
        new_node = self.Node(from_node.pos)
        d = new_node.calc_distance_to(to_node)

        new_node.path = [new_node.pos]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length // self.path_resolution)

        if n_expand > 0:
            steer_path = self.robot.inverse_dyn(new_node.pos, to_node.pos, n_expand)
            #use the end position to represent the current node and update the path
            new_node.pos = steer_path[-1]
            new_node.path += steer_path

        d = new_node.calc_distance_to(to_node)
        if d <= self.path_resolution:
            #this is considered as connectable
            new_node.path.append(to_node.pos)

            #so this position becomes the representation of this node
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
            rnd = self.Node(
                np.random.uniform(self.map.map_area[0], self.map.map_area[1])
                )
        else:  # goal point sampling
            rnd = self.Node(self.end.pos)
        return rnd

    def draw_graph(self, rnd=None):
        # plt.clf()
        # # for stopping simulation with the esc key.
        # plt.gcf().canvas.mpl_connect(
        #     'key_release_event',
        #     lambda event: [exit(0) if event.key == 'escape' else None])
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.pos[0], rnd.pos[1], "^k")

        # draw the map
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
        dlist = [ node.calc_distance_to(rnd_node)
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind


    def check_collision_free(self, node):
        if node is None:
            return False
        for p in node.path:
            if self.map.in_collision(np.array(p)):
                return False
        return True
    
def stop_robot():
    arlo.stop()
    sleep(0.041)

def turn_left():
    print("Turning left")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))
    sleep(1.2)
    stop_robot()

def turn_right():
    print("Turning right")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 0))
    sleep(1.2)
    stop_robot()

def go_forward():
    print("Going forwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    sleep(0.041)

def go_backward():
    print("Going backwards")
    print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))
    sleep(1)
    stop_robot()

def navigate_to_waypoints(path):
    for i in range(len(path) - 1):
        current_pos = np.array(path[i])
        next_pos = np.array(path[i + 1])

        direction = next_pos - current_pos
        distance = np.linalg.norm(direction)
        unit_direction = direction / distance  # Normalize to get direction only

        # Rotate to face the next waypoint
        angle_to_turn = np.arctan2(unit_direction[1], unit_direction[0])
        rotate_to_angle(angle_to_turn)

        # Move forward to the waypoint
        while np.linalg.norm(current_pos - next_pos) > thresholdDistance:
            go_forward()
            current_pos += unit_direction * leftSpeed * 0.041  # Adjust by speed and time to simulate movement

        stop_robot()

def rotate_to_angle(target_angle):
    # Dummy implementation of rotation to target angle
    # This function would ideally include code to measure the robot's current orientation
    print(f"Rotating to angle: {target_angle}")
    if target_angle > 0:
        turn_right()
    else:
        turn_left()
    stop_robot()
class MapFromNpy:
    """
    Map class that loads and processes map from 'mapGod.npy' file
    """

    def __init__(self, map_file):
        self.map_data = np.load(map_file)
        self.map_area = [(0, 0), (self.map_data.shape[1], self.map_data.shape[0])]
        self.extent = [0, self.map_data.shape[1], 0, self.map_data.shape[0]]

    def draw_map(self):
        plt.imshow(self.map_data, cmap='Blues', origin='lower', extent=self.extent)

    def in_collision(self, pos):
        x, y = int(pos[0]), int(pos[1])
        return self.map_data[y, x] == 1  # Assuming 1 is obstacle, 0 is free space


import robot_models

def main():
    path_res = 0.05

    # Use the new map loaded from "mapGod.npy"
    map = MapFromNpy('mapGod.npy')

    robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])

    rrt = RRT(
        start=[0, 49],         # Define your start position
        goal=[90, 49],        # Define your goal position
        robot_model=robot,
        map=map,              # Use the map loaded from the file
        expand_dis=2.5,
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
            print("found path!!")
            navigate_to_waypoints(path)

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