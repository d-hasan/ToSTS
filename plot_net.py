import matplotlib.pyplot as plt 
import numpy as np


def plot_boundary(edges):
    line_segments = []
    nodes = []
    for edge in edges:
        from_coord = edge.getFromNode().getCoord()
        to_coord = edge.getToNode().getCoord()
        nodes += [from_coord, to_coord]
        line_segment = [[from_coord[0], to_coord[0]], [from_coord[1], to_coord[1]]]
        line_segments.append(line_segment)
    
    nodes = np.array(nodes)
    # plt.scatter(nodes[:,0], nodes[:,1])
    # plt.show()

    for line_segment in line_segments:
        plt.plot(*line_segment, 'r')
        # break
    # plt.show()


def plot_nodes(nodes):
    coords = []
    for node in nodes:
        coord = node.getCoord()
        coords.append(coord)
    
    coords = np.array(coords)
   
    plt.scatter(coords[:,0], coords[:,1], s=8)

