import argparse
from argparse import REMAINDER
import os 

from tts_data import TTSData
from traffic_regions import TrafficRegions
from simulation_net import SimulationNet
from region_graph import RegionGraph
from trip_planner import TripPlanner


parser = argparse.ArgumentParser(description='Generate SUMO Trips from Toronto Transit Survey Data')
parser.add_argument('config', type=str, help='Config folder')
parser.add_argument('--sim-net-config', type=str, default='simulation_net.cfg', help='to change default simulation net config file')
parser.add_argument('--trip-planner-config', type=str, default='trip_planner.cfg', help='to change default trip planner config file')
parser.add_argument('--tts-data-config', type=str, default='tts_data.cfg', help='to change default tts data config file')
parser.add_argument('--traffic-regions-config', type=str, default='traffic_regions.cfg', help='to change default traffic regigons config file')



if __name__ == '__main__':
    args = parser.parse_args()

    config_dir = args.config

    print()

    print('Initializing Simulation Network...')
    sim_net_config = os.path.join(config_dir, args.sim_net_config)
    sim_net = SimulationNet(sim_net_config)
    print('Simulation Net initialized.\n')

    print('Initializing TTS Data...')
    tts_data_config = os.path.join(config_dir, args.tts_data_config)
    tts_data = TTSData(tts_data_config)
    print('TTS Data Initalized.\n')

    print('Initializing Traffic Regions...')
    traffic_regions_config = os.path.join(config_dir, args.traffic_regions_config)
    traffic_regions = TrafficRegions(traffic_regions_config, sim_net.crs,sim_net.location_offset)
    print('Initialized Traffic Regions.\n')

    print('Initializing Region Graph...')
    region_graph = RegionGraph(traffic_regions.pd_gdf, traffic_regions.region_gdf, traffic_regions.hways_gs)
    print('Initalized Region Graph.')

    print('Initializing Trip Planner...')
    trip_planner_config = os.path.join(config_dir, args.trip_planner_config)
    trip_planner = TripPlanner(trip_planner_config, sim_net, tts_data.smooth_time_data_sparse, traffic_regions, region_graph)
    print('Initialized Trip Planner.\n')
    
    trip_planner.generate_trips()








