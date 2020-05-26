import os
import sys
import pdb

import matplotlib.pyplot as plt 
import numpy as np

import plot_net

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import sumolib

net_path = 'toronto-data/osm.net.xml'
net = sumolib.net.readNet(net_path)

edges = net.getEdges()
nodes = net.getNodes()


def prune_edges(edges, coord_index, lower_coord=None, upper_coord=None, direction=1):
    ''' Removes edges that have coordinates outside a defined lower and upper bound, in the prescribed direction.

        edges (list): list of SUMO Edge objects pertaining to a side of the polygon boundary.
        coord_index (int): index of coordinate to use, 0: x-coord | 1:y-coord
        lower_coord (float): coordinate lower bound, optional if the road in question terminates at the lower coord.
        upper_coord (float): coordinate upper bound, optional if the road in question terminates at the upper coord.
        direction (int):  the directioon to consider the from/to of an edge, 1 denotes from < to and -1 denotes to < from. 
                        Used to enforce a counter-clockwise orientation to the polygon edges along the boundary.

    '''
    pruned_edges = []
    for edge in edges:
        from_coord = edge.getFromNode().getCoord()
        to_coord = edge.getToNode().getCoord()
        add_edge = False 

        if direction*from_coord[coord_index] < direction*to_coord[coord_index]:
            add_edge = True


        if lower_coord:
            if from_coord[coord_index] >= lower_coord[coord_index] and to_coord[coord_index] >= lower_coord[coord_index]:
                add_edge = add_edge & True 
            else:
                add_edge = add_edge & False

        if upper_coord:
            if from_coord[coord_index] <= upper_coord[coord_index] and to_coord[coord_index] <= upper_coord[coord_index]:
                add_edge = add_edge & True 
            else:
                add_edge = add_edge & False
        
        if add_edge:
            pruned_edges.append(edge)
        
    return pruned_edges


def get_south_boundary_edges():
    ''' Returns the edges for the southern boundary.
    '''
    # Intersectoin of bathurst and queens quay is the bottom-left polygon bound
    west_qq_node = net.getNode('cluster_1895192313_3466639218_5215359975_6247479486_6247479487_6247479489')
    west_coord = west_qq_node.getCoord()

    qq_edges = [edge for edge in edges if 'queens quay' in edge.getName().lower()]
    qq_edges = prune_edges(qq_edges, 0, west_coord)

    # Need the intersection of parliament with queens quay and lakeshore respectively
    south_parl_node = net.getNode('145493534')
    south_coord = south_parl_node.getCoord()
    north_parl_node = net.getNode('cluster_21090716_21099033')
    north_coord = north_parl_node.getCoord()

    parl_edges = [edge for edge in edges if 'parliament' in edge.getName().lower()]
    parl_edges = prune_edges(parl_edges, 1, south_coord, north_coord)
    
    # Lake shore between parliament and don roadway
    west_lake_node = net.getNode('cluster_21090716_21099033')
    west_coord = west_lake_node.getCoord()
    east_lake_node = net.getNode('cluster_404524713_404524716')
    east_coord = east_lake_node.getCoord()
    print(east_coord)

    lake_edges = [edge for edge in edges if 'lake shore' in edge.getName().lower()]
    lake_edges = prune_edges(lake_edges, 0, west_coord, east_coord)

    south_boundary_edges = qq_edges + parl_edges + lake_edges
    return south_boundary_edges


def get_north_boundary_edges():
    ''' Returns the edges for the northern boundary.
    '''
    # Intersection of bloor and bathurst
    west_node = net.getNode('21631714')
    west_coord = west_node.getCoord()

    # Artificial node where DVP and Bloor pass each other
    east_node = net.getNode('gneJ3')
    east_coord = east_node.getCoord()
    
    north_boundary_edges = [edge for edge in edges if 'bloor street' in edge.getName().lower()]
    north_boundary_edges = prune_edges(north_boundary_edges, 0, west_coord, east_coord, direction=-1)

    return north_boundary_edges


def get_west_boundary_edges():
    ''' Returns the edges for the western boundary.
    '''
    # Intersection of bloor and bathurst
    north_node = net.getNode('21631714')
    north_coord = north_node.getCoord()

    # Intersection of queens quay and bathurst
    south_node = net.getNode('cluster_1895192313_3466639218_5215359975_6247479486_6247479487_6247479489')
    south_coord = south_node.getCoord()

    west_boundary_edges = [edge for edge in edges if 'bathurst' in edge.getName().lower()]
    west_boundary_edges = prune_edges(west_boundary_edges, 1, south_coord, north_coord, direction=-1)

    return west_boundary_edges


def get_east_boundary_edges():
    ''' Returns the edges for the eastern boundary.
    '''
    # Aritificial bloor and don valley "intersection"
    north_node = net.getNode('gneJ3')
    north_coord = north_node.getCoord()

    # Don Valley and Don Roadway Intersection
    south_node = net.getNode('2258710645')
    south_coord = south_node.getCoord()

    dvp_edges = [edge for edge in edges if 'don valley' in edge.getName().lower()]
    dvp_edges = prune_edges(dvp_edges, 1, south_coord, north_coord)

    # Don Valley and Don Roadway Intersection
    north_node = net.getNode('2260363024')
    north_coord = north_node.getCoord()

    # Don Roadway and Lakeshore
    south_node = net.getNode('cluster_404524713_404524716')
    south_coord = south_node.getCoord()

    don_road_edges = [edge for edge in edges if 'don roadway' in edge.getName().lower()]
    don_road_edges = prune_edges(don_road_edges, 1, south_coord, north_coord)

    east_boundary_edges = dvp_edges + don_road_edges
    return east_boundary_edges


north_boundary_edges = get_north_boundary_edges()
west_boundary_edges = get_west_boundary_edges()
east_boundary_edges = get_east_boundary_edges()
south_boundary_edges = get_south_boundary_edges()

# need to connect DVP to Bloor
artificial_edge = sumolib.net.edge.Edge('fake_edge', net.getNode('699540'), net.getNode('gneJ3'), 0, None, 'fake')

all_edges = north_boundary_edges + west_boundary_edges + east_boundary_edges + south_boundary_edges + [artificial_edge]


def get_internal_nodes(boundary_edges):
    nodes = net.getNodes()

    internal_nodes = set()
    external_nodes = set()
    for node in nodes:
        crossings = 0
        x, y = node.getCoord()

        for edge in boundary_edges:
            from_node = edge.getFromNode()
            to_node = edge.getToNode()

            internal_nodes.add(from_node)
            internal_nodes.add(to_node)

            v1 = from_node.getCoord()
            v2 = to_node.getCoord()
            
            if (x >= v1[0]) != (x > v2[0]):
                m =  (v2[1] - v1[1]) / (v2[0] - v1[0])
                b = v1[1] - m*v1[0]

                if m*x + b >= y:
                    crossings += 1
                    
        if crossings % 2 == 1:
            internal_nodes.add(node)
        else:
            external_nodes.add(node)

    for node in internal_nodes.intersection(external_nodes):
        external_nodes.remove(node)
        
        
    print('Internal: {} | External: {}'.format(len(internal_nodes), len(external_nodes)))
    
    return internal_nodes, external_nodes


def get_terminal_nodes(internal_nodes, external_nodes):
    terminal_nodes = set()

    for node in external_nodes:
        neighbours = node.getNeighboringNodes()
        
        for neighbour in neighbours:
            if neighbour in internal_nodes:
                terminal_nodes.add(node)
                break 


    pruned_nodes = external_nodes.difference(terminal_nodes)

    print('Terminal : {} | Pruned: {}'.format(len(terminal_nodes), len(pruned_nodes)))
        
    return terminal_nodes, pruned_nodes



internal_nodes, external_nodes = get_internal_nodes(all_edges)
terminal_nodes, prune_nodes = get_terminal_nodes(internal_nodes, external_nodes)

# Plot the boundary, internal nodes, external nodes, and terminal nodes on the outside
plot_net.plot_boundary(all_edges)
plot_net.plot_nodes(internal_nodes)
plot_net.plot_nodes(external_nodes)
plot_net.plot_nodes(terminal_nodes)
plt.show()

# print(sumolib.geomhelper.positionAtShapeOffset(net.getLane('4652071#4_0').getShape(), 67.82))