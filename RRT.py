"""
Author(s):
Akhilrajan(Akhil) Vethirajan (v.akhilrajan@gmail.com)
Masters in Robotics,
University of Maryland, College Park
"""

from tqdm import tqdm
from utils.ArmConfig import *
from utils.FKSolver import *
from utils.VertexViz import *

# Manipulator Starting Config
q_init = [0, 0, 0, 0, 0, 0]
test_exploration = []


def euclidean_dist(P1, P2):
    dist1 = (P1[0] - P2[0]) ** 2
    dist2 = (P1[1] - P2[1]) ** 2
    dist3 = (P1[2] - P2[2]) ** 2
    return np.sqrt(dist1 + dist2 + dist3)


class Node:
    def __init__(self, q, parent=None):
        self.q = q
        self.parent = parent


class RRT:
    def __init__(self, q_init, q_goal, step_size, max_iter):
        self.q_init = q_init
        self.q_goal = q_goal
        self.step_size = step_size
        self.max_iter = max_iter

        self.vertices = [Node(q_init)]
        self.edges = []

    def explore(self):
        q_rand = create_config()  # Randomly sample a configuration
        nearest_vertex = self.find_nearest_vertex(q_rand)  # Find the nearest vertex
        # q_new = self.steer(nearest_vertex.q, q_rand)  # Move towards q_rand
        q_new = self.move_towards(nearest_vertex.q, q_rand)  # Move towards q_rand
        if self.obstacle_check_fn(q_new):  # Check if q_new is collision-free
            self.vertices.append(Node(q_new, parent=nearest_vertex))
            self.edges.append((nearest_vertex.q, q_new))
            test_exploration.append(forward_kinematics(self.vertices[-1].q))

    def find_nearest_vertex(self, q_rand):
        distances = [euclidean_dist(forward_kinematics(q_rand), forward_kinematics(vertex.q)) for vertex in self.vertices]
        nearest_vertex_index = np.argmin(distances)
        return self.vertices[nearest_vertex_index]

    # def nearest_vertex(self, q):
    #     nearest_vertex = self.vertices[0]
    #     min_distance = np.linalg.norm(forward_kinematics(q) - forward_kinematics(nearest_vertex.q))
    #     for v in self.vertices:
    #         d = np.linalg.norm(forward_kinematics(q) - forward_kinematics(v.q))
    #         if d < min_distance:
    #             nearest_vertex = v
    #             min_distance = d
    #     return nearest_vertex

    def steer(self, q_near, q_rand):
        q_new = q_near + self.step_size * (q_rand - q_near)
        return q_new

    def move_towards(self, start, goal):
        """ Move towards the goal node with a specific step size """
        delta = goal - start
        dist = np.linalg.norm(delta)
        if dist <= self.step_size:
            return goal
        else:
            unit_delta = delta / dist
            return start + unit_delta * self.step_size

    def obstacle_check_fn(self, q):
        return True

    def run(self):
        for i in tqdm(range(self.max_iter), desc='Running RRT', ncols=100, colour='BLUE'):
            self.explore()
            if euclidean_dist(forward_kinematics(self.vertices[-1].q), forward_kinematics(self.q_goal)) < self.step_size:
                # If we're close enough to the goal, add one final edge and return the path
                self.vertices.append(Node(self.q_goal, parent=self.vertices[-1]))
                self.edges.append((self.vertices[-1].q, self.q_goal))
                path = []
                node = self.vertices[-1]
                while node.parent is not None:
                    path.append(node.q)
                    node = node.parent
                path.append(node.q)
                print("Success : Path Found -->")
                return path[::-1]
        return None


if __name__ == '__main__':
    # Define the start and goal configurations
    q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # q_start = np.array([0.1, 0.0, -0.3, -1.0, 0.5, 0])
    # q_goal = np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2])
    # q_goal = np.array([5.5, 1.0, -0.5, -1.0, 0.5, 1.0])
    q_goal = np.array([-0.3, 0, -0.5, -1.0, 0.5, 0])
    # Initialize the RRT object
    rrt = RRT(q_start, q_goal, max_iter=1000, step_size=0.2)
    q_path = rrt.run()
    path = []
    for q in q_path:
        path.append(forward_kinematics(q))

    # Visualizer
    explored_vertex_viz(np.asarray(test_exploration), forward_kinematics(q_start), forward_kinematics(q_goal), path)






