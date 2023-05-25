import random
import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp

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
        # Extract obstacle information
        obstacles = env.get_obstacle_list()

        # Define the variables
        num_points = 50
        positions = cp.Variable((num_points, 2))
        velocities = cp.Variable((num_points - 1, 2))
        accelerations = cp.Variable((num_points - 2, 2))

        # Define the constraints
        constraints = [
            positions[0] == start_position,
            positions[-1] == end_position,
            velocities == cp.diff(positions, axis=0),
            accelerations == cp.diff(velocities, axis=0),
            cp.abs(velocities) <= 1.0,
            cp.abs(accelerations) <= 0.5
        ]

        # Define the objective function (minimize total acceleration)
        objective = cp.Minimize(cp.sum_squares(accelerations))

        # Define the problem
        problem = cp.Problem(objective, constraints)

        # Solve the problem
        problem.solve()

        if problem.status == cp.OPTIMAL:
            trajectory_positions = np.vstack((positions.value, end_position))
            self.visualize_trajectory(env, trajectory_positions)
        else:
            print("Planning failed.")

    def visualize_trajectory(self, env, positions):
        fig, ax = plt.subplots(figsize=(8,8))
        ax.set_xlim([0, env.width])
        ax.set_ylim([0, env.height])

        # Plot obstacles
        for obstacle in env.get_obstacle_list():
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='r')
            ax.add_patch(circle)

        # Plot trajectory on top of environment
        ax.plot(positions[:, 0], positions[:, 1], 'bo-')

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

quad_planner = Planner()
quad_planner.plan(env1, (1,1), (5,14))
quad_planner.plan(env2, (3,7), (1,14))