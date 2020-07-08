import os
import sys
import configparser
import subprocess
import pdb

import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np
import shapely
import pyproj
from shapely.geometry.linestring import LineString, Point


import plot_net
from traffic_regions import TrafficRegions

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import sumolib


class SimulationNet():

    def __init__(self, config_path):
        self.config = configparser.ConfigParser()
        self.config.read(config_path)


        self.net_path = self.config['paths']['net_path']
        self.net = sumolib.net.readNet(self.net_path)

        proj_str = self.net._location['projParameter']
        self.crs = pyproj.CRS(proj_str)
        self.location_offset = self.net.getLocationOffset()

        self.edges = self.net.getEdges()
        self.nodes = self.net.getNodes()

        self.card_directions = ['north', 'west', 'south', 'east']
        self.coord_indices = {'north': 0, 'south': 0, 'east': 1, 'west': 1}
        self.directions = {'north': -1, 'south': 1, 'east': 1, 'west': -1}

        all_boundary_edges = [self.get_boundary_edges(card_direction) for card_direction in self.card_directions]
        self.artificial_edges = self.get_artificial_edges()
        
        insertion_count = 0
        appended_boundary_edges = []
        for i, edges in enumerate(all_boundary_edges):
            appended_boundary_edges += edges 
            if self.artificial_edges[insertion_count][1] == i + 1:
                appended_boundary_edges += [self.artificial_edges[insertion_count][0]]
                insertion_count += 1
        
        self.boundary_edges = appended_boundary_edges
        
        self.boundary_polygon = self.create_polygon()

        self.internal_nodes, self.external_nodes = self.get_internal_nodes()
        self.terminal_nodes, self.pruned_nodes = self.get_terminal_nodes()
        self.outflow_nodes, self.outflow_nodes_major = self.get_outflow_nodes()
        self.inflow_nodes, self.inflow_nodes_major = self.get_inflow_nodes()

        self.edges_to_keep = self.get_edges_to_keep()
        self.save_edges_to_keep()
        self.prune_network()

        
    def create_polygon(self):
        polygon_vertices = [edge.getFromNode().getCoord() for edge in self.boundary_edges]
        boundary_polygon = shapely.geometry.Polygon(polygon_vertices)
        return boundary_polygon


    def prune_edges(self, edges, coord_index, lower_coord=None, upper_coord=None, direction=1):
        ''' Removes edges that have coordinates outside a defined lower and upper bound, in the prescribed direction.

            Parameters:
                edges (list): list of SUMO Edge objects pertaining to a side of the polygon boundary.
                coord_index (int): index of coordinate to use, 0: x-coord | 1:y-coord
                lower_coord (float): coordinate lower bound, optional if the road in question terminates at the lower coord.
                upper_coord (float): coordinate upper bound, optional if the road in question terminates at the upper coord.
                direction (int):  the directioon to consider the from/to of an edge, 1 denotes from < to and -1 denotes to < from. 
                                Used to enforce a counter-clockwise orientation to the polygon edges along the boundary.

            Returns:
                pruned_edges (set): set of edges not to be included in final SUMO network.
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
    
    # clarify that this only sorts them for one side of boundary
    def sort_edges_for_polygon(self, edges, coord_index, direction):
        from_coords = [edge.getFromNode().getCoord()[coord_index] * direction for edge in edges]
        zipped_lists = zip(from_coords, edges)
        sorted_pairs = sorted(zipped_lists)

        tuples = zip(*sorted_pairs)
        try:
            from_coords, edges = [list(tuple) for tuple in tuples]
        except:
            pdb.set_trace()

        return edges 


    def get_artificial_edges(self):
        edges = filter(None, [edge.strip().split(', ') for edge in self.config['artificial_edges']['edges'].strip().splitlines()])
        edges = list(edges)

        artificial_edges = []
        for i, edge in enumerate(edges):
            artificial_edge = sumolib.net.edge.Edge(
                'fake_edge_{}'.format(i), 
                self.net.getNode(edge[0]),
                self.net.getNode(edge[1]),
                0, None, 'fake'
                )
            artificial_edges.append([artificial_edge, int(edge[-1])])
        
        return artificial_edges


    def get_boundary_edges(self, card_direction):
        ''' Returns the edges for a given direction's boundary.
        '''
        coord_index = self.coord_indices[card_direction]
        direction = self.directions[card_direction]

        streets = list(filter(None, [street for street in self.config[card_direction]['streets'].strip().splitlines()]))
        nodes = list(filter(None, [node for node in self.config[card_direction]['nodes'].strip().splitlines()]))

        all_boundary_edges = []
        for i, street in enumerate(streets):
            start_node = self.net.getNode(nodes[i])
            start_coord = start_node.getCoord()

            end_node = self.net.getNode(nodes[i+1])
            end_coord = end_node.getCoord()
        
            boundary_edges = [edge for edge in self.edges if street in edge.getName().lower()]
            boundary_edges = self.prune_edges(boundary_edges, coord_index, start_coord, end_coord, direction=direction)
            all_boundary_edges += boundary_edges

        return all_boundary_edges

     
    # change implementatin to use shapely functions
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
            
        return terminal_nodes, pruned_nodes\


    def get_outflow_nodes(self):
        outflow_nodes = {}
        outflow_nodes_major = {}

        for node in self.terminal_nodes:
            incoming_edges = node.getIncoming()
            outflow_edges = [edge for edge in incoming_edges if edge.getFromNode() in self.internal_nodes]

            if len(outflow_edges) > 0:
                outflow_edge_capacity = sum([edge.getLaneNumber() * edge.getSpeed() for edge in outflow_edges])
                outflow_nodes[node] = outflow_edge_capacity
                if any([edge.getLaneNumber() > 1 for edge in outflow_edges]):
                    outflow_nodes_major[node] = outflow_edge_capacity
        
        return outflow_nodes, outflow_nodes_major

    
    def get_inflow_nodes(self):
        inflow_nodes = {}
        inflow_nodes_major = {}

        for node in self.terminal_nodes:
            outgoing_edges = node.getOutgoing()
            inflow_edges = [edge for edge in outgoing_edges if edge.getToNode() in self.internal_nodes]

            if len(inflow_edges) > 0:
                inflow_edge_capacity = sum([edge.getLaneNumber() * edge.getSpeed() for edge in inflow_edges])
                inflow_nodes[node] = inflow_edge_capacity
                if any([edge.getLaneNumber() > 1 for edge in inflow_edges]):
                    inflow_nodes_major[node] = inflow_edge_capacity

        return inflow_nodes, inflow_nodes_major


    def get_edges_to_keep(self):
        edges_to_keep = set()

        for edge in self.edges:
            from_node = edge.getFromNode()
            to_node = edge.getToNode()

            if from_node not in self.pruned_nodes or to_node not in self.pruned_nodes:
                edges_to_keep.add(edge)

        return edges_to_keep



    def save_edges_to_keep(self):
        path = self.config['paths']['edges_path']
        edge_ids_to_keep = [edge.getID() for edge in self.edges_to_keep]
        with open(path, 'w') as f:
            for edge in edge_ids_to_keep:
                f.write(edge + '\n')


    def prune_network(self):
        net_path = self.config['paths']['net_path']
        pruned_path = self.config['paths']['pruned_path']
        edges_path = self.config['paths']['edges_path']

        command = ['netconvert', '-s', net_path, '--keep-edges.input-file', edges_path, '-o', pruned_path]
        subprocess.call(command)


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



    