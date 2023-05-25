import random
import numpy as np
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, num_obs, width, height):
        self.width = width
        self.height = height
        self.obstacles = []

        # randomly generate obstacles
        for i in range(num_obs):
            x = np.random.uniform(0, self.width)
            y = np.random.uniform(0, self.height)
            radius = np.random.uniform(0.5, 2.0)
            self.obstacles.append((x, y, radius))

    def add_obstacle(self, position, radius):
        self.obstacles.append((position[0], position[1], radius))

    def remove_obstacle(self, position, radius):
        for obstacle in self.obstacles:
            if np.linalg.norm(np.array(obstacle[:2]) - np.array(position)) <= obstacle[2] + radius:
                self.obstacles.remove(obstacle)

    def get_obstacle_list(self):
        return self.obstacles

    def is_obstacle(self, point):
        for obstacle in self.obstacles:
            if np.linalg.norm(np.array(obstacle[:2]) - np.array(point)) <= obstacle[2]:
                return True
        return False

    def visualize_environment(self):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim([0, self.width])
        ax.set_ylim([0, self.height])

        # Plot obstacles
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='r')
            ax.add_patch(circle)

        plt.show()

class RRTPlanner:
    def __init__(self, step_size=0.5, max_iterations=1000):
        self.step_size = step_size
        self.max_iterations = max_iterations

    def plan(self, env, start_position, goal_position):
        self.goal_position = goal_position
        nodes = [start_position]

        for _ in range(self.max_iterations):
            random_point = self._generate_random_point(env)
            nearest_node = self._find_nearest_node(nodes, random_point)
            new_node = self._expand_towards(nearest_node, random_point)
            
            if not env.is_obstacle(new_node):
                nodes.append(new_node)
                if self._is_goal_reached(new_node):
                    path = self._construct_path(nodes)
                    self.visualize_path(env, path)
                    return path

        print("Planning failed.")
        return None

    def _generate_random_point(self, env):
        if random.random() < 0.1:  # Small probability
            return np.array(self.goal_position)
        else:
            return np.array([random.uniform(0, env.width), random.uniform(0, env.height)])

    def _find_nearest_node(self, nodes, point):
        distances = [np.linalg.norm(np.array(node) - point) for node in nodes]
        nearest_index = np.argmin(distances)
        return nodes[nearest_index]

    def _expand_towards(self, start, goal):
        direction = goal - start
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            return goal
        else:
            unit_direction = direction / distance
            new_point = start + self.step_size * unit_direction
            return new_point

    def _is_goal_reached(self, point):
        return np.linalg.norm(np.array(point) - np.array(self.goal_position)) <= self.step_size

    def _construct_path(self, nodes):
        path = [self.goal_position]
        current_node = self.goal_position
        while current_node != nodes[0]:
            distances = [np.linalg.norm(np.array(node) - current_node) for node in nodes]
            min_distance_index = np.argmin(distances)
            current_node = nodes[min_distance_index]
            path.append(current_node)
        path.reverse()
        return path

    def visualize_path(self, env, path):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim([0, env.width])
        ax.set_ylim([0, env.height])

        # Plot obstacles
        for obstacle in env.get_obstacle_list():
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='r')
            ax.add_patch(circle)

        plt.plot([point[0] for point in path], [point[1] for point in path], 'bo-')
        plt.show()

# Testing
# Environment 1
env1 = Environment(num_obs=7, width=10, height=15)
env1.add_obstacle((4, 8), 1.5)
env1.remove_obstacle((7, 5), 1.0)

print(env1.get_obstacle_list())

env1.visualize_environment()

# Environment 2
env2 = Environment(num_obs=4, width=10, height=15)
env2.add_obstacle((4, 8), 1.5)
env2.remove_obstacle((7, 5), 1.0)

print(env2.get_obstacle_list())

env2.visualize_environment()

rrt_planner = RRTPlanner(step_size=0.5, max_iterations=1000)
rrt_planner.plan(env1, (1, 1), (5, 14))
rrt_planner.plan(env2, (3, 7), (1, 14))
