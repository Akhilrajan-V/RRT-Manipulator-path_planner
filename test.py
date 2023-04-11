import numpy as np
import random

class Node:
    def __init__(self, q, parent=None):
        self.q = q
        self.parent = parent

class RRT:
    def __init__(self, q_init, q_goal, step_size, max_iter, obstacle_check_fn):
        self.q_init = q_init
        self.q_goal = q_goal
        self.step_size = step_size
        self.max_iter = max_iter
        self.obstacle_check_fn = obstacle_check_fn

        self.vertices = [Node(q_init)]
        self.edges = []

    def extend(self):
        q_rand = np.random.rand(6) * 2 * np.pi - np.pi # Randomly sample a configuration
        q_near = self.nearest_vertex(q_rand).q # Find the nearest vertex
        q_new = self.steer(q_near, q_rand) # Steer towards q_rand
        # if self.obstacle_check_fn(q_new): # Check if q_new is collision-free
        self.vertices.append(Node(q_new, q_near))
        self.edges.append((q_near, q_new))

    def nearest_vertex(self, q):
        nearest_vertex = self.vertices[0]
        min_distance = np.linalg.norm(q - nearest_vertex.q)
        for v in self.vertices:
            d = np.linalg.norm(q - v.q)
            if d < min_distance:
                nearest_vertex = v
                min_distance = d
        return nearest_vertex

    def steer(self, q_near, q_rand):
        q_new = q_near + self.step_size * (q_rand - q_near)
        return q_new

    def find_path(self):
        for i in range(self.max_iter):
            print("{} of {}".format(i+1,self.max_iter))
            self.extend()
            if np.linalg.norm(self.vertices[-1].q - self.q_goal) < 0.8:
                # If we're close enough to the goal, add one final edge and return the path
                self.vertices.append(Node(self.q_goal))
                self.edges.append((self.vertices[-1].q, self.q_goal))
                path = []
                node = self.vertices[-1]
                while node.parent is not None:
                    path.append(node.q)
                    node = node.parent
                path.append(node.q)
                # print(self.vertices[0].q for v in self.vertices)
                return path[::-1]
        return None


# Define the start and goal configurations
q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q_goal = np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2])
q_goal = np.array([5.5, 1.0, -0.5, -1.0, 0.5, 1.0])

# Define the bounds of the workspace
bounds = np.array([[-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi]])

# Initialize the RRT object
rrt = RRT(q_start, q_goal,  max_iter=10000, step_size=1, obstacle_check_fn=1)

# Run the RRT algorithm
path = rrt.find_path()

# Print the resulting path
print(path)