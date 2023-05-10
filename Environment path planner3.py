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
        fig, ax = plt.subplots(figsize=(8,8))
        ax.set_xlim([0, self.width])
        ax.set_ylim([0, self.height])

        # Plot obstacles
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='r')
            ax.add_patch(circle)

        plt.show()

class Planner:
    def __init__(self):
        pass

    def plan(self, env, start_position, end_position):
        # Your path planning algorithm here
        pass

# Example usage
env = Environment(num_obs=7, width=10, height=15)
env.add_obstacle((4, 8), 1.5)
env.remove_obstacle((7, 5), 1.0)

print(env.get_obstacle_list())

env.visualize_environment()

quad_planner = Planner()
quad_planner.plan(env, (1,1), (5,14))