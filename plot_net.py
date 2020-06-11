import xml.etree.ElementTree as ET
import os 
import sys

import matplotlib.pyplot as plt 
import numpy as np

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import sumolib

def plot_boundary(ax, edges):
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
        ax.plot(*line_segment, 'r', label='boundary edges')
        # break
    # plt.show()


def plot_nodes(ax, nodes, marker, label, color):
    coords = []
    for node in nodes:
        coord = node.getCoord()
        coords.append(coord)
    
    coords = np.array(coords)
   
    ax.scatter(coords[:,0], coords[:,1], s=18, marker=marker, label=label, color=color, zorder=2)


def plot_interior_edges(ax, edges, internal_nodes):
    line_segments = []
    nodes = []
    for edge in edges:
        from_node = edge.getFromNode()
        to_node = edge.getToNode()

        if from_node in internal_nodes or to_node in internal_nodes:
            from_coord = from_node.getCoord()
            to_coord = to_node.getCoord()
            nodes += [from_coord, to_coord]
            line_segment = [[from_coord[0], to_coord[0]], [from_coord[1], to_coord[1]]]
            line_segments.append(line_segment)
    
    nodes = np.array(nodes)
    # plt.scatter(nodes[:,0], nodes[:,1])
    # plt.show()

    for line_segment in line_segments:
        ax.plot(*line_segment, 'tab:gray', zorder=1, label='edges')

def plot_parking_lots(ax, net, parking_lot_xml):
    tree = ET.parse(parking_lot_xml)
    root = tree.getroot()

    parking_lots = [child.attrib for child in root]
    x_locations = []
    y_locations = []

    for lot in parking_lots:
        lane = lot['lane']
        position = (float(lot['endPos']) + float(lot['startPos']))/2
        x, y = sumolib.geomhelper.positionAtShapeOffset(net.getLane(lane).getShape(), position)
        x_locations.append(x)
        y_locations.append(y)
    
    ax.scatter(x_locations, y_locations, s=12, marker='D', label='parking lot', color='r', zorder=2)
