import random 
import sys 
import os 
import pdb 
import time 
import subprocess
import xml.etree.ElementTree as ET
import string 


import numpy as np 
import scipy
from networkx.algorithms import shortest_paths
from shapely.geometry import LineString, Point, MultiPoint 
from shapely import ops

from utils import convert_clock_to_minutes

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import sumolib

import utils

xml_version=  '<?xml version="1.0" encoding="UTF-8"?>\n'
    # trip_header = '<trips xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/trips_file.xsd">\n'
trip_header = '<trips>\n'



class TripTemplate():
    trip_template = string.Template(
        '\t\t<trip id="$trip_id" \
from="$from_edge" departPos="$from_pos" \
to="$to_edge" arrivalPos="$to_pos" \
fromJunction="$from_node" toJunction="$to_node" \
inflowTime="$inflow_time" />\n')

    
    def __init__(self, origin, dest, inflow_time, trip_num):
        self.origin_edge = origin[0]
        self.origin_edge_pos = origin[1]
        self.origin_node = origin[2]
        self.origin_point = origin[3]
        self.origin_taz_original = origin[4]
        
        self.dest_edge = dest[0]
        self.dest_edge_pos = dest[1]
        self.dest_node = dest[2]
        self.dest_point = dest[3]
        self.dest_taz_original = dest[4]

        self.inflow_time = inflow_time 
        self.trip_num = trip_num
    
        self.trip_string = self.construct_trip_string()


    def construct_trip_string(self):
        trip_id = '{}-{}-{}'.format(
            self.origin_taz_original, 
            self.dest_taz_original, 
            self.trip_num
        )
        trip_string = self.trip_template.substitute(
            trip_id=trip_id,
            from_edge=self.origin_edge.getID(),
            from_pos=self.origin_edge_pos,
            to_edge=self.dest_edge.getID(),
            to_pos=self.dest_edge_pos,
            from_node=self.origin_node.getID(),
            to_node=self.dest_node.getID(),
            inflow_time=self.inflow_time    
        )
        return trip_string 


class TripSamplesGenerator():
    
    def __init__(self, config_path, sim_net, tts_trips, traffic_regions, region_graph):
        super().__init__()

        self.config = utils.get_config(config_path)
        
        self.hway_inflow = self.config['hway_inflow']
        self.hway_outflow = self.config['hway_outflow']

        self.avg_trivial_speed = int(self.config['travel_speed']['trivial'])* 1000/3600
        self.avg_hway_speed = int(self.config['travel_speed']['hway']) * 1000/3600
        self.min_speed_factor = float(self.config['travel_speed']['min_factor'])
        self.max_speed_factor = float(self.config['travel_speed']['max_factor'])
        self.speed_dev = float(self.config['travel_speed']['dev'])

        self.trip_start_time = int(self.config['trip']['start_time'])
        self.trip_end_time = int(self.config['trip']['end_time'])
        self.trip_start_time_minutes = utils.convert_clock_to_minutes(int(self.config['trip']['start_time']))
        
        self.min_intersection = float(self.config['trip']['min_intersection'])
        self.max_node_distance = int(self.config['trip']['max_node_distance'])
        self.num_close_nodes = int(self.config['trip']['num_close_nodes'])
        self.trivial_path_length = int(self.config['trip']['trivial_path_length'])
        self.max_sample_size = int(self.config['trip']['max_sample_size'])

        self.output_file = self.config['DEFAULT']['output_file']
        self.net_path = self.config['DEFAULT']['net_path']

        self.sim_net = sim_net
        self.aoi_boundary = self.sim_net.boundary_polygon
        self.trips = tts_trips
        self.traffic_regions = traffic_regions
        self.taz_gdf = self.traffic_regions.taz_gdf
        self.region_graph = region_graph
        self.aoi_taz = self.get_aoi_taz()
        self.aoi_boundary_taz = self.get_aoi_boundary_taz()
        self.aoi_pds = self.get_aoi_pds()
        self.aoi_regions = self.get_aoi_regions()
        self.aoi_hways = self.get_aoi_hways()

        self.speed_distr = self.get_speed_distribution()

        self.trip_id_counter = 0
        self.total_trips = 0
        self.trip_type_counts = [0, 0, 0, 0]
        self.ignored_trips = 0

        self.trivial_trips = 0
        self.hway_trips = 0

        self.sumo_edge_time = 0
        self.sumo_edge_count = 0

        self.trivial_inflow_travel_count = 0
        self.trivial_inflow_travel_time = 0 

        self.hway_inflow_travel_count = 0
        self.hway_inflow_travel_time = 0

        self.generate_point_count = 0
        self.generate_point_time = 0

        self.hway_trip_cache = {}
        self.trip_distance_cache = {}
        self.pd_borders_cache = {}
        self.hway_in_pd_cache = {}
        self.hway_nearest_points_cache = {}

        


    def get_aoi_taz(self):
        taz_gdf = self.traffic_regions.taz_gdf
        intersecting_index = [self.aoi_boundary.intersects(taz) for taz in taz_gdf.geometry]
        aoi_taz = taz_gdf[intersecting_index]
        aoi_taz = set(list(aoi_taz.index))
        return aoi_taz

    def get_aoi_boundary_taz(self):
        taz_gdf = self.traffic_regions.taz_gdf
        intersecting_index = [self.aoi_boundary.exterior.intersects(taz) for taz in taz_gdf.geometry]
        aoi_boundary_taz = taz_gdf[intersecting_index]
        aoi_boundary_taz = set(list(aoi_boundary_taz.index))
        return aoi_boundary_taz
    
    def get_aoi_regions(self):
        region_gdf = self.traffic_regions.region_gdf
        intersecting_index = [self.aoi_boundary.intersects(region) for region in region_gdf.geometry]
        aoi_regions = region_gdf[intersecting_index]
        aoi_regions = set(list(aoi_regions.index))
        return aoi_regions

    def get_aoi_hways(self):
        aoi_boundary = self.aoi_boundary
        aoi_hways = set()
        hways = self.traffic_regions.hways_gs
        for name, hway in hways.iteritems():
            if hway.intersects(aoi_boundary):
                aoi_hways.add(name)

        aoi_hways.add('trivial')

        return aoi_hways

    def get_aoi_pds(self):
        ''' Hardcoded to return only PD 1, as that contains most of the AOI (98.8% is contained in PD1)
        '''
        # pd_gdf = self.traffic_regions.pd_gdf
        # intersecting_index = [self.aoi_boundary.intersects(plan_dist) for plan_dist in pd_gdf.geometry]
        # aoi_pds = pd_gdf[intersecting_index]
        # aoi_pds = set(list(aoi_pds.index))
        aoi_pds = {1}
        return aoi_pds


    def initialize_trip_file(self):
        # if os.path.exists(self.output_file):
        #     os.remove(self.output_file)
        with open(self.output_file, 'w') as f:
            f.write(xml_version)
            f.write(trip_header)


    def write_trips_to_file(self, orig_dest, sumo_trips):
        origin, dest = orig_dest

        orig_dest_string = '\t<OD origin="{}" dest="{}">\n'.format(origin, dest)

        trip_strings = [sumo_trip.trip_string for sumo_trip in sumo_trips]
        trip_strings = ''.join(trip_strings)
        with open(self.output_file, 'a') as f:
            f.write(orig_dest_string)
            f.write(trip_strings)   
            f.write('\t</OD>\n') 


    def complete_trip_file(self):
        with open(self.output_file, 'a') as f:
            f.write('</trips>')


    def get_speed_distribution(self):
        mean = 1
        a = (self.min_speed_factor - mean)/self.speed_dev
        b = (self.max_speed_factor - mean)/self.speed_dev
        return scipy.stats.truncnorm(a, b, loc=mean, scale=self.speed_dev)

    def get_trivial_speed(self):
        return self.speed_distr.rvs() * self.avg_trivial_speed

    def get_hway_speed(self):
        return self.speed_distr.rvs() * self.avg_hway_speed

    
    def sort_trips(self):
        tree = ET.parse(self.output_file)
        root = tree.getroot()

        def sortchildrenby(parent, attr):
            parent[:] = sorted(parent, key=lambda child: int(child.get(attr)))

        sortchildrenby(root, 'depart')

        tree.write(self.output_file)


    def validate_trips(self):
        trip_path = self.output_file
        net_path = self.net_path

        command = [
            'duarouter',
            '--repair',
            'repair.from',
            'repair.to',
            '--ignore-errors',
            '--write-trips',
            '-n', 
            '--route-files', trip_path,
            '-o', net_path 
        ]

    def generate_trips(self, od_pairs=None):
        print('Generating OD Trips to Sample')

        self.initialize_trip_file()

        trips_processed = 0
        start_time = time.time() 

        if not od_pairs:
            od_pairs = list(self.trips.keys())

        all_trip_samples = {}

        for i, orig_dest in enumerate(od_pairs):
            trip_count = self.trips[orig_dest]
            trip = [orig_dest[0], orig_dest[1], trip_count]
            if trip_count > self.max_sample_size:
                trip[-1] = self.max_sample_size 

            trip_type = self.classify_trip(trip)
            if trip_type == -1:
                continue
            elif trip_type == 0b00:
                sumo_trips = self.trip_ext_to_ext(trip)
                
            elif trip_type == 0b01:
                sumo_trips = self.trip_ext_to_int(trip)
                
            elif trip_type == 0b10:
                sumo_trips = self.trip_int_to_ext(trip)

            elif trip_type == 0b11:
                sumo_trips = self.trip_int_to_int(trip)

            trips_processed += trip[2]

            if sumo_trips:
                self.trip_type_counts[trip_type] += len(sumo_trips)
                elapsed_time = time.time() - start_time
                print('\tGenerated {} trips, {} OD pairs in {:.1f}s'.format(trips_processed, i+1, elapsed_time))
                print('\t\t E-E:{} | E-I:{} | I-E:{} | I-I:{}'.format(*self.trip_type_counts))
                print('\t\t Ignored Trips: {}'.format(self.ignored_trips))
                print('\t\t Trivial Trips: {} | Hway Trips: {}'.format(
                    self.trivial_trips, 
                    self.hway_trips
                ))

                all_trip_samples[orig_dest] = sumo_trips
                self.write_trips_to_file(orig_dest, sumo_trips)

            else:
                self.ignored_trips += trip[2]

            

                    
    def classify_trip(self, od_data):
        trip_type = 0b00
        origin = od_data[0]
        dest = od_data[1]
        
        if origin in self.taz_gdf.index and dest in self.taz_gdf.index:
            if origin in self.aoi_taz:
                trip_type = trip_type | 0b10
            if dest in self.aoi_taz:
                trip_type = trip_type | 0b01
        else:
            trip_type = -1
            
        return trip_type
    
    def trip_ext_to_ext(self, trip):
        # pdb.set_trace()
        trip_count = trip[2]
        
        origin_taz = self.taz_gdf.loc[trip[0]]
        origin_region = origin_taz.REGION
        
        dest_taz = self.taz_gdf.loc[trip[1]] 
        dest_region = dest_taz.REGION
        
        path = self.region_graph.pd_paths[origin_taz.PD][dest_taz.PD]
       
        if origin_region == dest_region and origin_region not in self.aoi_regions:
            return None 
        elif len(path) <= self.trivial_path_length:
            # The planning districts are second degree neighbours within the AOI Regions
            # 50% chance to take a straight line trip
            # 50% chance to take a hway trip 
            get_trip = self.get_short_external_trip
        else:
            # Travel between regions defaults to long trip
            get_trip = self.get_long_external_trip
        
        all_trips = []
        for i in range(trip_count):
            trip = get_trip(origin_taz, dest_taz, i+1)
            if trip:
                all_trips.append(trip)

        if len(all_trips) > 0:
            return all_trips
        else:
            return None
    
    def get_short_external_trip(self, origin_taz, dest_taz, trip_num):
        origin_point = self.generate_point_in_polygon(origin_taz.geometry)
        dest_point = self.generate_point_in_polygon(dest_taz.geometry)

        boundary = self.aoi_boundary
        straight_trip = LineString([origin_point, dest_point])
        internal_trip_segment = straight_trip.intersection(boundary)

        inflow_edge, outflow_edge = None, None 
        
        if internal_trip_segment.length/straight_trip.length > self.min_intersection:
            external_trip_segments = ops.split(straight_trip, boundary)
            try:
                inflow_trip_segment = external_trip_segments[0]
                outflow_trip_segment = external_trip_segments[-1]
            except:
                pdb.set_trace()
                return None 
            
            inflow_edge, inflow_node = self.get_trivial_inflow_edge(inflow_trip_segment, major_inflow=False)
            inflow_time = self.compute_trivial_inflow_travel_time(origin_taz, inflow_node)
            # print('Departure: {} | Inflow: {}'.format(departure_time, inflow_time))

            outflow_edge, outflow_node = self.get_trivial_outflow_edge(outflow_trip_segment, major_outflow=False)

        
        if inflow_edge and outflow_edge:
            self.trivial_trips += 1
            self.trip_id_counter += 1
            origin_data = [inflow_edge, 0, inflow_node, origin_point, origin_taz.name]
            dest_data = [outflow_edge, 'max', outflow_node, dest_point, dest_taz.name]
            return TripTemplate(origin_data, dest_data, inflow_time, self.trip_id_counter)
        else: 
            return None 


    def get_long_external_trip(self, origin_taz, dest_taz, trip_num):
        origin_point = self.generate_point_in_polygon(origin_taz.geometry)
        dest_point = self.generate_point_in_polygon(dest_taz.geometry)

        # Will return None if the hway trip does not go through AOI nodes
        hway_trip = self.get_hway_trip(origin_taz, dest_taz)
        if hway_trip:
            trip_nodes, trip_edges, aoi_trip_edges = hway_trip
        else:
            return None 

        inflow_edge, outflow_edge = None, None             

        inflow_edge, inflow_node  = self.get_inflow_edge(
                    trip_nodes, 
                    trip_edges, 
                    aoi_trip_edges, 
                    origin_point, 
                    dest_point,
                    ext_to_ext=True  
                )
        inflow_time = self.compute_hway_inflow_travel_time(trip_nodes, trip_edges, aoi_trip_edges, origin_point, inflow_node)
        # print('Departure: {} | Inflow: {}'.format(departure_time, inflow_time))
        if inflow_edge:
            inflow_point = inflow_edge.getFromNode().getCoord()
            # treat the outflow_edge like an internal to external trip, with the internal point being the inflow node
            outflow_edge, outflow_node = self.get_outflow_edge(trip_nodes, trip_edges, aoi_trip_edges, inflow_point, dest_point, ext_to_ext=True)

        
        if inflow_edge and outflow_edge:
            self.hway_trips += 1
            self.trip_id_counter += 1
            origin_data = [inflow_edge, 0, inflow_node, origin_point, origin_taz.name]
            dest_data = [outflow_edge, 'max', outflow_node, dest_point, dest_taz.name]
            return TripTemplate(origin_data, dest_data, inflow_time, self.trip_id_counter)
        return None 
    
    
    def trip_ext_to_int(self, trip):
        trip_count = trip[2]
        
        origin_taz = self.taz_gdf.loc[trip[0]]
        origin_region = origin_taz.REGION
        
        dest_taz = self.taz_gdf.loc[trip[1]] 
        dest_region = dest_taz.REGION

        path = self.region_graph.pd_paths[origin_taz.PD][dest_taz.PD]

        all_trips = []
        for i in range(trip_count):
            origin_point = self.generate_point_in_polygon(origin_taz.geometry)
            dest_point = self.generate_point_in_polygon(dest_taz.geometry)

            if not Point(dest_point).within(self.aoi_boundary):
                continue

            dest_edge, dest_edge_pos, dest_node = self.get_sumo_edge_for_point(dest_point, incoming=True)
            dest_data = [dest_edge, dest_edge_pos, dest_node, dest_point, trip[1]]
            
            # if (len(path) < 3 and random.random() < 0.5) or len(path) == 1:
            if len(path) <= self.trivial_path_length:
                self.trivial_trips += 1
                boundary = self.aoi_boundary
                straight_trip = LineString([Point(origin_point), Point(dest_point)])
                inflow_edge, inflow_node = self.get_trivial_inflow_edge(straight_trip,major_inflow=False)
                inflow_time = self.compute_trivial_inflow_travel_time(origin_point, inflow_node)
            else:
                hway_trip = self.get_hway_trip(origin_taz, dest_taz)
                if hway_trip:
                    trip_nodes, trip_edges, aoi_trip_edges = hway_trip 
                else:
                    # no valid hway trip through AOI
                    continue 

                inflow_edge, inflow_node = self.get_inflow_edge(
                    trip_nodes, 
                    trip_edges, 
                    aoi_trip_edges, 
                    origin_point, 
                    dest_point,
                )
                if not inflow_node:
                    pdb.set_trace()
                inflow_time = self.compute_hway_inflow_travel_time(trip_nodes, trip_edges, aoi_trip_edges, origin_point, inflow_node)
                # print('Departure: {} | Inflow: {}'.format(departure_times[i], inflow_time))
                self.hway_trips += 1
            
            inflow_data = [inflow_edge, 0, inflow_node, origin_point, trip[0]]
            
            self.trip_id_counter += 1
            all_trips.append(TripTemplate(inflow_data, dest_data, inflow_time, self.trip_id_counter))
         
        if len(all_trips) > 0:
            return all_trips
        else:
            return None


    def trip_int_to_ext(self, trip):
        trip_count = trip[2]
        origin_taz = self.taz_gdf.loc[trip[0]]
        dest_taz = self.taz_gdf.loc[trip[1]]
        
        path = self.region_graph.pd_paths[origin_taz.PD][dest_taz.PD]


        all_trips = []
        for i in range(trip_count):    
            origin_point = self.generate_point_in_polygon(origin_taz.geometry)
            dest_point = self.generate_point_in_polygon(dest_taz.geometry)

            if not Point(origin_point).within(self.aoi_boundary):
                continue 

            origin_edge, origin_edge_pos, origin_node = self.get_sumo_edge_for_point(origin_point, outgoing=True)
            origin_data = [origin_edge, origin_edge_pos, origin_node, origin_point, trip[0]]
            
            if len(path) <= self.trivial_path_length:
                self.trivial_trips += 1
                straight_trip = LineString([Point(origin_point), Point(dest_point)])
                outflow_edge, outflow_node = self.get_trivial_outflow_edge(straight_trip, major_outflow=False)
            else:
                hway_trip = self.get_hway_trip(origin_taz, dest_taz)
                if hway_trip:
                    trip_nodes, trip_edges, aoi_trip_edges = hway_trip 
                else: 
                    # hway trip doesnt go through AOI 
                    continue 

                outflow_edge, outflow_node = self.get_outflow_edge(trip_nodes, trip_edges, aoi_trip_edges, origin_point, dest_point)
                self.hway_trips += 1


            dest_data = [outflow_edge, 'max', outflow_node, dest_point, trip[1]]

            self.trip_id_counter += 1
            # print('Departure: {} '.format(departure_times[i]))
            all_trips.append(TripTemplate(origin_data, dest_data, 0, self.trip_id_counter))
            
        return all_trips
    

    def trip_int_to_int(self, trip):
        trip_count = trip[2]
        origin_taz = self.taz_gdf.loc[trip[0]]
        dest_taz = self.taz_gdf.loc[trip[1]]

        all_trips = []
        for i in range(trip_count):
            self.trivial_trips += 1
            origin_point = self.generate_point_in_polygon(origin_taz.geometry)
            dest_point = self.generate_point_in_polygon(dest_taz.geometry)

            if not Point(origin_point).within(self.aoi_boundary) or not Point(dest_point).within(self.aoi_boundary):
                continue

            origin_edge, origin_edge_pos, origin_node = self.get_sumo_edge_for_point(origin_point, outgoing=True)
            origin_data = [origin_edge, origin_edge_pos, origin_node, origin_point, trip[0]]
        
            dest_edge, dest_edge_pos, dest_node = self.get_sumo_edge_for_point(dest_point, incoming=True)
            dest_data = [dest_edge, dest_edge_pos, dest_node, dest_point, trip[1]]


            self.trip_id_counter += 1
            # print('Departure: {} '.format(departure_times[i]))
            sumo_trip = TripTemplate(origin_data, dest_data, 0, self.trip_id_counter)
                           
            all_trips.append(sumo_trip)
            
        return all_trips

    
    def get_hway_trip(self, origin_taz, dest_taz):
        if (origin_taz.name, dest_taz.name) in self.hway_trip_cache:
            return self.hway_trip_cache[(origin_taz.name, dest_taz.name)]
        else:
            return self.compute_hway_trip(origin_taz, dest_taz)


    def compute_hway_trip(self, origin_taz, dest_taz):
        trip_nodes = self.region_graph.pd_paths[origin_taz.PD][dest_taz.PD]

        if len([node for node in trip_nodes if node in self.aoi_pds]) > 0:
            trip_edges = []
            for from_node, to_node in zip(trip_nodes[:-1], trip_nodes[1:]):
                edges = self.region_graph.pd_graph.get_edge_data(from_node, to_node)
                hways = [edge.split('-')[-1] for edge in edges if 'trivial' not in edge]

                if len(hways) == 0: 
                    hways = ['trivial']

                previous_hway = trip_edges[-1][-1] if len(trip_edges) > 0 else None
                if previous_hway and previous_hway in hways:
                    # select the hway that was previously used 
                    trip_edges.append([from_node, to_node, previous_hway])
                else:
                    # pick any travel edge to take
                    trip_edges.append([from_node, to_node, random.choice(hways)])

            aoi_trip_edges = []
            # Compute the trip edges entering and exiting from AOI
            for i, edge_data in enumerate(trip_edges):
                from_node, to_node, edge = edge_data
                
                if from_node in self.aoi_pds or to_node in self.aoi_pds:
                    aoi_trip_edges.append(i)

            if len(aoi_trip_edges) > 0:
                self.hway_trip_cache[(origin_taz.name, dest_taz.name)] = [trip_nodes, trip_edges, aoi_trip_edges]
                return trip_nodes, trip_edges, aoi_trip_edges

        self.hway_trip_cache[(origin_taz.name, dest_taz.name)] = None      
        
        return None


    def get_inflow_edge(self, trip_nodes, trip_edges, aoi_trip_edges, origin_point, dest_point, ext_to_ext=False):
        inflow_edge = None 
        inflow_node = None 

        edge_data = trip_edges[aoi_trip_edges[0]]
        from_node, to_node, edge = edge_data 

        if edge == 'trivial':
            if not ext_to_ext:
                pdb.set_trace()
            else:
                pdb.set_trace()
            return None, 0
        else:
            inflow_edge, inflow_node = self.get_hway_inflow_edge(edge)
            return inflow_edge, inflow_node


    def get_hway_inflow_edge(self, hway_name):
        ''' Returns highway inflow edge, given a hway name.
        '''
        inflow_node = self.sim_net.net.getNode(self.hway_inflow[hway_name])
        
        inflow_edges = [edge for edge in inflow_node.getOutgoing() 
                if edge.getToNode() not in self.sim_net.pruned_nodes]
        inflow_edge = random.choice(inflow_edges)

        return inflow_edge, inflow_node


    def get_trivial_inflow_edge(self, inflow_geometry, major_inflow):
        ''' Computes the closest inflow edge on the sumo network to an "inflow geometry".
            Inflow geometries can be shapely Point or LineString objects.
            Points represent a point near the AOI boundary.
            LineStrings represents a straight line trip between two points, that crosses the AOI boundary.
        '''
        all_inflow_nodes = self.sim_net.inflow_nodes_major if major_inflow else self.sim_net.inflow_nodes

        inflow_dists = [(inflow_geometry.distance(Point(node.getCoord())), node) 
                            for node in all_inflow_nodes]
        inflow_dists_bounded = [(dist, node) for dist, node in inflow_dists if dist < self.max_node_distance]
        inflow_dists_closest = sorted(inflow_dists_bounded)[:self.num_close_nodes]

        inflow_node_capacities = np.array([all_inflow_nodes[node] for _, node in inflow_dists_closest])
        inflow_node_capacities = inflow_node_capacities/inflow_node_capacities.sum() 

        if len(inflow_dists_closest) > 0:
            inflow_index = np.random.choice(np.arange(len(inflow_dists_closest)), p=inflow_node_capacities)
            inflow_dist, inflow_node = inflow_dists_closest[inflow_index]
        else:
            return None, None 

        inflow_edges = [edge for edge in inflow_node.getOutgoing() 
                        if edge.getToNode() in self.sim_net.internal_nodes]
        inflow_edge = random.choice(inflow_edges)

        return inflow_edge, inflow_node


    def get_outflow_edge(self, trip_nodes, trip_edges, aoi_trip_edges, origin_point, dest_point, ext_to_ext=False): 
        outflow_edge = None 

        edge_data = trip_edges[aoi_trip_edges[-1]]
        from_node, to_node, edge = edge_data 

        if edge == 'trivial':
            if not ext_to_ext:
                pdb.set_trace()
            return None, 0
        else:
            outflow_edge, outflow_node = self.get_hway_outflow_edge(edge)
            return outflow_edge, outflow_node
           

    def get_trivial_outflow_edge(self, outflow_geometry, major_outflow):
        all_outflow_nodes = self.sim_net.outflow_nodes_major if major_outflow else self.sim_net.outflow_nodes
        outflow_dists = [(outflow_geometry.distance(Point(node.getCoord())), node) for node in all_outflow_nodes]
        outflow_dists_bounded = [(dist, node) for dist, node in outflow_dists if dist < self.max_node_distance]
        outflow_dists_closest = sorted(outflow_dists_bounded)[:self.num_close_nodes]

        outflow_node_capacities = np.array([all_outflow_nodes[node] for _, node in outflow_dists_closest])
        outflow_node_capacities = outflow_node_capacities/outflow_node_capacities.sum()

        if len(outflow_dists_closest) > 0:
            outflow_index = np.random.choice(np.arange(len(outflow_dists_closest)), p=outflow_node_capacities)
            outflow_dist, outflow_node = outflow_dists_closest[outflow_index]
        else:
            return None, None 

        outflow_edges = [edge for edge in outflow_node.getIncoming()
                        if edge.getFromNode() in self.sim_net.internal_nodes]
        outflow_edge = random.choice(outflow_edges)

        return outflow_edge, outflow_node
        

    def get_hway_outflow_edge(self, hway_name):
        ''' Returns highway outflow edge, given a hway name.
        '''
        outflow_node = self.sim_net.net.getNode(self.hway_outflow[hway_name])
        
        outflow_edges = [edge for edge in outflow_node.getIncoming() 
                if edge.getToNode() not in self.sim_net.pruned_nodes]
        outflow_edge = random.choice(outflow_edges)

        return outflow_edge, outflow_node


    def get_sumo_edge_for_point(self, point, incoming=False, outgoing=False):
        edges = []
        iteration = 0
        max_iterations = 7
        while len(edges) == 0 and iteration < max_iterations:
            radius = 25 * (2 ** iteration)
            edges = self.sim_net.net.getNeighboringEdges(*point, r=radius)
            iteration += 1
            
        if len(edges) > 0:
            dist_edges = sorted(edges, key=lambda close_edge: close_edge[1])
            closest_edge, distance = dist_edges[0]
            edge_pos = sumolib.geomhelper.polygonOffsetWithMinimumDistanceToPoint(point, closest_edge.getShape())
        else:
            import pdb; pdb.set_trace()

        node_distances = [(Point(node.getCoord()).distance(Point(point)), node)
                        for node in self.sim_net.internal_nodes 
                        if (len(node.getOutgoing()) > 0 and outgoing)
                        or (len(node.getIncoming()) > 0 and incoming)]
        closest_node = sorted(node_distances)[0][1]


        return closest_edge, edge_pos, closest_node

    def compute_hway_inflow_travel_time(self, trip_nodes, trip_edges, aoi_trip_edges, origin_point, inflow_node):

        prior_inflow_edges = trip_edges[:aoi_trip_edges[0]+1]

        hway_travel_time = 0
        trivial_travel_time = 0

        if len(prior_inflow_edges) == 1:
            trivial_travel_time += self.compute_trivial_inflow_travel_time(origin_point, inflow_node)
        else:

            # Easiest solution, just compute straight line
            # inflow_point = inflow_node.getCoord()
            # distance = Point(origin_point).distance(Point(inflow_point))
            # travel_time = (avg_hway_travel_speed * distance) / 60 # conversion to minutes
            # return travel_time

            
            from_node, to_node, edge = prior_inflow_edges[0]

            if edge == 'trivial':
                border = self.compute_pd_border(from_node, to_node)
                try:
                    midpoint = border.interpolate(0.5, normalized=True)
                except:
                    midpoint = border.centroid
                distance = Point(origin_point).distance(midpoint)
                trivial_travel_time += distance / self.avg_trivial_speed

            else: 
                border = self.compute_pd_border(from_node, to_node)
                hway_in_curr_pd = self.compute_hway_in_pd(edge, to_node)

                distance_from_origin = Point(origin_point).distance(hway_in_curr_pd)
                trivial_travel_time += distance_from_origin / self.avg_trivial_speed

                closest_point_to_origin = ops.nearest_points(hway_in_curr_pd, Point(origin_point))[0]
                closest_point_to_border = ops.nearest_points(hway_in_curr_pd, border)[0]
                hway_legs = ops.split(hway_in_curr_pd, MultiPoint([closest_point_to_origin, closest_point_to_border]))
                hway_leg = hway_legs[1] if len(hway_legs) > 1 else hway_legs[0]
                # pdb.set_trace()
                distance_on_hway = hway_leg.length
                hway_travel_time += distance_on_hway / self.avg_hway_speed
            
            
            for i, edge_data in enumerate(prior_inflow_edges[1:], 1):
                prev_node, curr_node, prev_edge = prior_inflow_edges[i-1] 
                _, next_node, next_edge = edge_data

                trip_key = (prev_node, curr_node, next_node)

                if trip_key not in self.trip_distance_cache:
                    curr_pd_centroid = self.traffic_regions.pd_gdf.loc[curr_node].geometry.centroid

                    prev_border = self.compute_pd_border(prev_node, curr_node)
                    next_border = self.compute_pd_border(curr_node, next_node)
                    
                    prev_distance = curr_pd_centroid.distance(prev_border)
                    next_distance = curr_pd_centroid.distance(next_border)

                    self.trip_distance_cache[trip_key] = [prev_distance, next_distance]
                


                prev_distance, next_distance = self.trip_distance_cache[trip_key]
                travel_time = 0
                if prev_edge == 'trivial':
                    trivial_travel_time += prev_distance / self.avg_trivial_speed
                else:
                    hway_travel_time += prev_distance / self.avg_hway_speed
                
                if next_edge == 'trivial':
                    trivial_travel_time += next_distance / self.avg_trivial_speed
                else:
                    hway_travel_time += next_distance / self.avg_hway_speed


        total_travel_time = hway_travel_time + trivial_travel_time


        return total_travel_time


    def compute_travel_on_hways_in_plan_dist(self, prev_hway_name, next_hway_name, from_pd_name, curr_pd_name, next_pd_name):
        prev_hway_in_pd = self.compute_hway_in_pd(prev_hway_name, curr_pd_name)
        next_hway_in_pd = self.compute_hway_in_pd(next_hway_name, curr_pd_name)

        hways_closest_points = ops.nearest_points(prev_hway_in_pd, next_hway_in_pd)
        distance_between = hways_closest_points[0].distance(hways_closest_points[1])

        prev_border = self.compute_pd_border(from_pd_name, curr_pd_name)
        prev_hway_closest_prev_border = ops.nearest_points(prev_hway_in_pd, prev_border)
        prev_hway_split_points = MultiPoint([hways_closest_points[0], prev_hway_closest_prev_border[0]])
        prev_hway_distance = ops.split(prev_hway_in_pd, prev_hway_split_points)
        pdb.set_trace()


        next_border = self.compute_pd_border(curr_pd_name, next_pd_name)
        next_hway_closest_next_border = ops.nearest_points(next_hway_in_pd, next_border)[0]
        next_hway_split_points = MultiPoint([hways_closest_points[1], next_hway_closest_next_border])
        next_hway_distance = ops.split(next_hway_in_pd, next_hway_split_points)
        
        return prev_hway_distance + next_hway_distance, distance_between

    def compute_travel_between_borders(self, pd_name_1, pd_name_2, pd_name_3):
        border_1 = self.compute_pd_border(pd_name_1, pd_name_2)
        border_2 = self.compute_pd_border(pd_name_2, pd_name_3)

        midpoint_1 = border_1.interpolate(0.5, normalized=True)
        midpoint_2 = border_2.interpolate(0.5, normalized=True)

        distance = midpoint_1.distance(midpoint_2)

        return distance 


    def compute_trivial_inflow_travel_time(self, origin_point, inflow_node):
        inflow_point = inflow_node.getCoord()
        distance = Point(origin_point).distance(Point(inflow_point))
        travel_time = distance / self.avg_trivial_speed

        return travel_time


    def compute_pd_border(self, pd_name_1, pd_name_2):
        key = tuple(sorted([pd_name_1, pd_name_2]))

        if key in self.pd_borders_cache:
            border = self.pd_borders_cache[key]
        else:
            pd_1 = self.traffic_regions.pd_gdf.loc[pd_name_1].geometry.buffer(0)
            pd_2 = self.traffic_regions.pd_gdf.loc[pd_name_2].geometry.buffer(0)

            border = pd_1.intersection(pd_2)
            self.pd_borders_cache[key] = border 

        return  border 

    def compute_hway_in_pd(self, hway_name, pd_name):
        key = (hway_name, pd_name)
        if not key in self.hway_in_pd_cache:
            hway = self.traffic_regions.hways_gs.loc[hway_name]
            curr_pd = self.traffic_regions.pd_gdf.loc[pd_name].geometry.buffer(0)

            hway_in_pd = hway.intersection(curr_pd)
            self.hway_in_pd_cache[key] = hway_in_pd

        hway_in_pd = self.hway_in_pd_cache[key]
        return hway_in_pd

    
    def generate_point_in_polygon(self, polygon):

        min_x, min_y, max_x, max_y = polygon.bounds
        point_inside = False
        while not point_inside:
            x = np.random.rand() * (max_x - min_x) + min_x
            y = np.random.rand() * (max_y - min_y) + min_y
            point = Point(x, y)
            if point.within(polygon):
                point_inside = True

        return (x, y)
        
        


