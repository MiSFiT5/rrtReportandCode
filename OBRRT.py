import os
import sys
import math
import numpy as np
import plotting, utils
import env
import time

showAnimation = True
obstacle_sampling_rate = 0.4

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, environment, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = environment
        if showAnimation:
            self.plotting = plotting.Plotting(self.env, s_start, s_goal)
        self.utils = utils.Utils(self.env)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        iter_goal = None
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and iter_goal == None and not self.utils.is_collision(node_new, self.s_goal):
                    node_new = self.new_state(node_new, self.s_goal)
                    node_goal = node_new
                    iter_goal = i

        if iter_goal == None:
            return None, self.iter_max
        else:
            return self.extract_path(node_goal), iter_goal

    def generate_random_node(self, goal_sample_rate):
        if np.random.random() < goal_sample_rate:
            return self.s_goal

        delta = self.utils.delta

        if np.random.random() < obstacle_sampling_rate:
            # Randomly choose an obstacle to sample a point from
            obs = self.env.obs_rectangle[np.random.randint(len(self.env.obs_rectangle))]
            x, y, w, h = obs

            # List of corner points of the obstacle
            corners = [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]

            # Introduce a bias towards obstacle corners
            corner = corners[np.random.randint(4)]
            buffer = 2.0  # Buffer to avoid sampling too close to the obstacle
            x_min = max(self.x_range[0] + delta, corner[0] - buffer)
            x_max = min(self.x_range[1] - delta, corner[0] + buffer)
            y_min = max(self.y_range[0] + delta, corner[1] - buffer)
            y_max = min(self.y_range[1] - delta, corner[1] + buffer)

            while True:
                rand_x = np.random.uniform(x_min, x_max)
                rand_y = np.random.uniform(y_min, y_max)
                node = Node((rand_x, rand_y))
                if not self.utils.is_inside_obs(node):
                    return node

        # Fallback: Uniform random sampling if not sampling near obstacles
        return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def get_path_length(path):
    length = 0
    for i, k in zip(path[0::], path[1::]):
        length += np.linalg.norm(np.array(i) - np.array(k))
    return length


def main():
    start_time = time.time()
    x_start = (2, 2)
    x_goal = (49, 24)
    environment = env.Env2()

    rrt = Rrt(environment, x_start, x_goal, 2, 0.10, 2500)
    path, nb_iter = rrt.planning()

    length = None

    if path:
        print('Found path in ' + str(nb_iter) + ' iterations, length : ' + str(get_path_length(path)))
        length = get_path_length(path)
        if showAnimation:
            rrt.plotting.animation(rrt.vertex, path, f"RRT with obstacle_sampling_rate={obstacle_sampling_rate*100}%", True)
            plotting.plt.show()
    else:
        print("No Path Found in " + str(nb_iter) + " iterations!")
        if showAnimation:
            rrt.plotting.animation(rrt.vertex, [], f"RRT with obstacle_sampling_rate={obstacle_sampling_rate*100}%", True)
            plotting.plt.show()
    
    print("Execution time: %s seconds" % (time.time() - start_time))

    return length, (time.time() - start_time)

if __name__ == '__main__':
    length, times = [], []
    for i in range(1):
        l, t = main()
        length.append(l) if l else None
        times.append(t)
    
    print("Average path length: ", np.mean(length))
    print("Average execution time: ", np.mean(times))
