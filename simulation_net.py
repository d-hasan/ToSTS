import os
import sys
import pdb

import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np
import shapely
import pyproj


import plot_net
from traffic_regions import TrafficRegions

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import sumolib


class SimulationNet():

    def __init__(self, net_path):
        self.net_path = net_path
        self.net = sumolib.net.readNet(self.net_path)

        proj_str = self.net._location['projParameter']
        self.crs = pyproj.CRS(proj_str)
        self.location_offset = self.net.getLocationOffset()

        self.edges = self.net.getEdges()
        self.nodes = self.net.getNodes()

        self.coord_indices = {'north': 0, 'south': 0, 'east': 1, 'west': 1}
        self.directions = {'north': -1, 'south': 1, 'east': 1, 'west': -1}

        self.north_boundary_edges = self.get_north_boundary_edges()
        self.west_boundary_edges = self.get_west_boundary_edges()
        self.east_boundary_edges = self.get_east_boundary_edges()
        self.south_boundary_edges = self.get_south_boundary_edges()

        self.artificial_edge = sumolib.net.edge.Edge('fake_edge', self.net.getNode('699540'), self.net.getNode('gneJ3'), 0, None, 'fake')

        self.boundary_edges = self.north_boundary_edges + self.west_boundary_edges + self.south_boundary_edges + self.east_boundary_edges + [self.artificial_edge]
        
        self.boundary_polygon = self.create_polygon()

        self.internal_nodes, self.external_nodes = self.get_internal_nodes()
        self.terminal_nodes, self.pruned_nodes = self.get_terminal_nodes()



    def create_polygon(self):
        polygon_vertices = [edge.getFromNode().getCoord() for edge in self.boundary_edges]
        boundary_polygon = shapely.geometry.Polygon(polygon_vertices)
        return boundary_polygon


    def prune_edges(self, edges, coord_index, lower_coord=None, upper_coord=None, direction=1):
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

        pruned_edges = self.sort_edges_for_polygon(pruned_edges, coord_index, direction)
            
        return pruned_edges
    

    def sort_edges_for_polygon(self, edges, coord_index, direction):
        from_coords = [edge.getFromNode().getCoord()[coord_index] * direction for edge in edges]
        zipped_lists = zip(from_coords, edges)
        sorted_pairs = sorted(zipped_lists)

        tuples = zip(*sorted_pairs)
        from_coords, edges = [list(tuple) for tuple in tuples]

        return edges 


    def get_south_boundary_edges(self):
        ''' Returns the edges for the southern boundary.
        '''
        coord_index = self.coord_indices['south']
        direction = self.directions['south']

        # Intersectoin of bathurst and queens quay is the bottom-left polygon bound
        west_qq_node = self.net.getNode('cluster_1895192313_3466639218_5215359975_6247479486_6247479487_6247479489')
        west_coord = west_qq_node.getCoord()

        qq_edges = [edge for edge in self.edges if 'queens quay' in edge.getName().lower()]
        qq_edges = self.prune_edges(qq_edges, coord_index, west_coord, direction=direction)

        # Need the intersection of parliament with queens quay and lakeshore respectively
        south_parl_node = self.net.getNode('145493534')
        south_coord = south_parl_node.getCoord()
        north_parl_node = self.net.getNode('cluster_21090716_21099033')
        north_coord = north_parl_node.getCoord()

        parl_edges = [edge for edge in self.edges if 'parliament' in edge.getName().lower()]
        parl_edges = self.prune_edges(parl_edges, coord_index, south_coord, north_coord, direction=direction)
        
        # Lake shore between parliament and don roadway
        west_lake_node = self.net.getNode('cluster_21090716_21099033')
        west_coord = west_lake_node.getCoord()
        east_lake_node = self.net.getNode('cluster_404524713_404524716')
        east_coord = east_lake_node.getCoord()

        lake_edges = [edge for edge in self.edges if 'lake shore' in edge.getName().lower()]
        lake_edges = self.prune_edges(lake_edges, coord_index, west_coord, east_coord, direction=direction)

        south_boundary_edges = qq_edges + parl_edges + lake_edges
        return south_boundary_edges


    def get_north_boundary_edges(self):
        ''' Returns the edges for the northern boundary.
        '''
        coord_index = self.coord_indices['north']
        direction = self.directions['north']

        # Intersection of bloor and bathurst
        west_node = self.net.getNode('21631714')
        west_coord = west_node.getCoord()

        # Artificial node where DVP and Bloor pass each other
        east_node = self.net.getNode('gneJ3')
        east_coord = east_node.getCoord()
        
        north_boundary_edges = [edge for edge in self.edges if 'bloor street' in edge.getName().lower()]
        north_boundary_edges = self.prune_edges(north_boundary_edges, coord_index, west_coord, east_coord, direction=direction)

        return north_boundary_edges


    def get_west_boundary_edges(self):
        ''' Returns the edges for the western boundary.
        '''
        coord_index = self.coord_indices['west']
        direction = self.directions['west']

        # Intersection of bloor and bathurst
        north_node = self.net.getNode('21631714')
        north_coord = north_node.getCoord()

        # Intersection of queens quay and bathurst
        south_node = self.net.getNode('cluster_1895192313_3466639218_5215359975_6247479486_6247479487_6247479489')
        south_coord = south_node.getCoord()

        west_boundary_edges = [edge for edge in self.edges if 'bathurst' in edge.getName().lower()]
        west_boundary_edges = self.prune_edges(west_boundary_edges, coord_index, south_coord, north_coord, direction=direction)

        return west_boundary_edges


    def get_east_boundary_edges(self):
        ''' Returns the edges for the eastern boundary.
        '''
        coord_index = self.coord_indices['east']
        direction = self.directions['east']

        # Aritificial bloor and don valley "intersection"
        north_node = self.net.getNode('gneJ3')
        north_coord = north_node.getCoord()

        # Don Valley and Don Roadway Intersection
        south_node = self.net.getNode('2258710645')
        south_coord = south_node.getCoord()

        dvp_edges = [edge for edge in self.edges if 'don valley' in edge.getName().lower()]
        dvp_edges = self.prune_edges(dvp_edges, coord_index, south_coord, north_coord, direction=direction)

        # Don Valley and Don Roadway Intersection
        north_node = self.net.getNode('2260363024')
        north_coord = north_node.getCoord()

        # Don Roadway and Lakeshore
        south_node = self.net.getNode('cluster_404524713_404524716')
        south_coord = south_node.getCoord()

        don_road_edges = [edge for edge in self.edges if 'don roadway' in edge.getName().lower()]
        don_road_edges = self.prune_edges(don_road_edges, coord_index, south_coord, north_coord, direction=direction)

        east_boundary_edges = don_road_edges + dvp_edges 
        return east_boundary_edges


    def get_internal_nodes(self):
        nodes = self.net.getNodes()

        internal_nodes = set()
        external_nodes = set()
        for node in nodes:
            crossings = 0
            x, y = node.getCoord()

            for edge in self.boundary_edges:
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


    def get_terminal_nodes(self):
        terminal_nodes = set()

        for node in self.external_nodes:
            neighbours = node.getNeighboringNodes()
            
            for neighbour in neighbours:
                if neighbour in self.internal_nodes:
                    terminal_nodes.add(node)
                    break 


        pruned_nodes = self.external_nodes.difference(terminal_nodes)

        print('Terminal : {} | Pruned: {}'.format(len(terminal_nodes), len(pruned_nodes)))
            
        return terminal_nodes, pruned_nodes


    def plot_clean_net(self, ax=None):
        if ax is None:
            figure, ax = plt.subplots(figsize=(15, 15))
        
        plot_net.plot_interior_edges(ax, self.edges, self.internal_nodes)
        plot_net.plot_boundary(ax, self.boundary_edges)
        # plot_net.plot_nodes(ax, external_nodes, 'o', 'external nodes')
        plot_net.plot_nodes(ax, self.internal_nodes, 'o', 'network nodes', 'xkcd:green')
        plot_net.plot_nodes(ax, self.terminal_nodes, 'o', 'inflow/outflow nodes', 'orange')
        # plot_net.plot_parking_lots(ax, net, parking_lot_xml)

        custom_lines = [Line2D([0], [0], marker='o', color='w', label='Scatter', markerfacecolor='b', markersize=12),
                        Line2D([0], [0], marker='o', color='w', label='Scatter', markerfacecolor='orange', markersize=12),
                        Line2D([0], [0], color='tab:gray', lw=2),
                        # Line2D([0], [0], color='r', lw=2)
                        ]

        # plt.legend(custom_lines, ['Intersection Nodes', 'Inflows/Outflows', 'Edges'])

        ax.grid(False)
        ax.set_ylabel('metres')
        ax.set_xlabel('metres')
        ax.set_title('Toronto Downtown Road Network')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(-500, 7500)
        ax.set_ylim(-500, 6000)


        return ax 



    