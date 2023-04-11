"""
Author(s):
Akhilrajan(Akhil) Vethirajan (v.akhilrajan@gmail.com)
Masters in Robotics,
University of Maryland, College Park
"""

import matplotlib.pyplot as plt


def explored_vertex_viz(vertexes, start, goal, path):
    # Generate some random 3D points
    points = vertexes

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')

    # Plot the points
    for i in range(len(vertexes)):
        ax.scatter(points[i][0], points[i][1], points[i][2])
    # Set the axes labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    ax = fig.add_subplot(122, projection='3d')
    for i in range(len(path) - 1):
        ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], [path[i][2], path[i+1][2]], c='r')
    ax.scatter(goal[0], goal[1], goal[2], marker='v', c='g', label="Goal")
    ax.scatter(start[0], start[1], start[2], marker='v', c='b', label="Start")

    # Set the axes labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    # Show the plot
    plt.show()
