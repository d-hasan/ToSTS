import sys 
import os 
import pdb 
import time 
import subprocess
import xml.etree.ElementTree as ET
import string 


import numpy as np 
import scipy 


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


class SUMOTrip():
    trip_template_junction = string.Template(
    '\t<trip id="$trip_id" depart="$departure_time" from="$from_edge" \
    departPos="$from_pos" to="$to_edge" arrivalPos="$to_pos" \
    fromJunction="$from_node" toJunction="$to_node"/>\n')

    trip_template_no_junction = string.Template(
    '\t<trip id="$trip_id" depart="$departure_time" from="$from_edge" \
    departPos="$from_pos" to="$to_edge" arrivalPos="$to_pos"/>\n')

    def __init__(self, sample_trip, departure_time, trip_num):
        self.sample_trip = sample_trip
        self.departure_time = departure_time
        self.trip_id = '{}-{}-{}'.format(
            self.sample_trip['id'],
            departure_time,
            trip_num
        )

        self.junctions = True     
        self._validate_sample_trip()
    
        self.trip_string = self.construct_trip_string()

    def _validate_sample_trip(self):
        if 'departPos' not in self.sample_trip:
            self.sample_trip['departPos'] = 'random'
        if 'arrivalPos' not in self.sample_trip:
            self.sample_trip['arrivalPos'] = 'random'
        if 'fromJunction' not in self.sample_trip:
            self.junctions = False 
        
    def construct_trip_string(self):
        if self.junctions:
            trip_string = self.trip_template_junction.substitute(
                trip_id=self.trip_id,
                departure_time=self.departure_time, 
                from_edge=self.sample_trip['from'],
                from_pos=self.sample_trip['departPos'],
                to_edge=self.sample_trip['to'],
                to_pos=self.sample_trip['arrivalPos'],
                from_node=self.sample_trip['fromJunction'],
                to_node=self.sample_trip['toJunction']
            )
        else:
            trip_string = self.trip_template_no_junction.substitute(
                trip_id=self.trip_id,
                departure_time=self.departure_time, 
                from_edge=self.sample_trip['from'],
                from_pos=self.sample_trip['departPos'],
                to_edge=self.sample_trip['to'],
                to_pos=self.sample_trip['arrivalPos']
            )
        return trip_string 


class TripSampler():
    
    def __init__(self, config_path):
        super().__init__()

        self.config = utils.get_config(config_path)

        self.avg_trivial_speed = int(self.config['travel_speed']['trivial'])* 1000/3600
        self.avg_hway_speed = int(self.config['travel_speed']['hway']) * 1000/3600
        self.min_speed_factor = float(self.config['travel_speed']['min_factor'])
        self.max_speed_factor = float(self.config['travel_speed']['max_factor'])
        self.speed_dev = float(self.config['travel_speed']['dev'])

        self.trip_start_time = int(self.config['trip']['start_time'])
        self.trip_end_time = int(self.config['trip']['end_time'])
        self.trip_start_time_minutes = utils.convert_clock_to_minutes(int(self.config['trip']['start_time']))
        self.time_interval = int(self.config['trip']['time_interval']) * 60
        
        self.net_path = self.config['DEFAULT']['net_path']
        self.trip_samples_path = self.config['DEFAULT']['trip_samples_path']
        self.raw_file_path = 'temp_trips.rou.xml'


        self.trip_samples = self.load_trip_samples()
        # self.trips = tts_trips
        self.speed_distr = self.get_speed_distribution()

        self.trip_id_counter = 0
        self.total_trips = 0
        self.trip_type_counts = [0, 0, 0, 0]
        self.ignored_trips = 0

        self.trivial_trips = 0
        self.hway_trips = 0

        


    def load_trip_samples(self):
        tree = ET.parse(self.trip_samples_path)
        root = tree.getroot()
        
        sample_trips = {}

        for OD in root.iter('OD'):
            od_attrib = OD.attrib
            origin, dest = od_attrib['origin'], od_attrib['dest']

            sample_trips[(int(origin), int(dest))] = OD
        
        return sample_trips


    def initialize_trip_file(self, file_path):
        # if os.path.exists(self.output_file):
        #     os.remove(self.output_file)
        with open(file_path, 'w') as f:
            f.write(xml_version)
            f.write(trip_header)


    def write_trips_to_file(self, file_path, sumo_trips):
        trip_strings = [sumo_trip.trip_string for sumo_trip in sumo_trips]
        trip_strings = ''.join(trip_strings)
        with open(file_path, 'a') as f:
            f.write(trip_strings)    


    def complete_trip_file(self, file_path):
        with open(file_path, 'a') as f:
            f.write('</trips>')

    def get_speed_distribution(self):
        mean = 1
        a = (self.min_speed_factor - mean)/self.speed_dev
        b = (self.max_speed_factor - mean)/self.speed_dev
        return scipy.stats.truncnorm(a, b, loc=mean, scale=self.speed_dev)


    def sort_trips(self, file_path):
        print('Sorting trips by departure time for SUMO.')
        tree = ET.parse(file_path)
        root = tree.getroot()

        def sortchildrenby(parent, attr):
            parent[:] = sorted(parent, key=lambda child: float(child.get(attr)))

        sortchildrenby(root, 'depart')

        tree.write(file_path)


    def validate_trips(self, trip_path):
        validation_start = time.time()
        print('Validating Trips with DUAROUTER.')
        command = [
            'duarouter',
            '--repair',
            '--ignore-errors',
            '--write-trips',
            '--no-warnings',
            '--bulk-routing',
            '--route-files', trip_path,
            '-n', self.net_path,
            '-o', self.raw_file_path
        ]

        process = subprocess.Popen(command, stdout=subprocess.PIPE)
        stdout = process.communicate()[0]
        print('STDOUT:{}'.format(stdout))
        subprocess.call(command)
        validation_elapsed = time.time() - validation_start
        print('Validation Time: {:.1f}s'.format(validation_elapsed))
        os.rename(self.raw_file_path, trip_path)

    def generate_trips(self, file_path, trip_distribution):
        print('Generating Trips: {} to {}'.format(self.trip_start_time, self.trip_end_time))

        self.initialize_trip_file(file_path)

        total_trips_processed = 0
        prints_completed = 0
        elapsed_start = time.time() 
        od_pairs = list(self.trip_samples.keys())

        for i, od_pair in enumerate(od_pairs):
            trips_processed = 0
            od_sample_trips = self.trip_samples[od_pair]
            pair_trips = []
            times = sorted(list(trip_distribution[od_pair].keys()))

            times = [t for t in times if t >= self.trip_start_time]
            if self.trip_end_time > 0:
                times = [t for t in times if t <= self.trip_end_time]

            for start_time in times:
                start_time_minutes = utils.convert_clock_to_minutes(start_time)
                start_time_minutes = start_time_minutes - self.trip_start_time_minutes
                start_time_seconds = start_time_minutes * 60

                trip_count = trip_distribution[od_pair][start_time]
                sumo_trips = self.sample_trips(
                    start_time_seconds, 
                    trip_count, 
                    od_sample_trips,
                    total_trips_processed
                )
                pair_trips += sumo_trips

                total_trips_processed += len(sumo_trips)
                trips_processed += len(sumo_trips)

            self.write_trips_to_file(file_path, pair_trips)

            if total_trips_processed > 25000 * (prints_completed+1):
                elapsed_time = time.time() - elapsed_start
                print('\tGenerated {} OD Pairs, {} Total Trips in {:.1f}s '.format(
                    i+1, 
                    total_trips_processed,
                    elapsed_time,
                ))
                prints_completed += 1
            

        self.complete_trip_file(file_path)
        self.sort_trips(file_path)
        self.validate_trips(file_path)

        

    def sample_trips(self, start_time, trip_count, od_sample_trips, total_trips_processed):
        lambda_ = trip_count / (self.time_interval/60)
        # pdb.set_trace()
        beta = 1/lambda_

        departure_intervals = np.random.exponential(beta, trip_count)*60
        # departure_intervals = np.arange(0, self.time_interval, beta)
        departure_times = start_time + departure_intervals.cumsum()
        departure_times = departure_times[departure_times < start_time + self.time_interval]

        sumo_trips = []
        # pdb.set_trace()

        for i in range(len(departure_times)):
            index = np.random.choice(len(od_sample_trips))
            sample_trip = od_sample_trips[index].attrib 

            speed_factor = self.speed_distr.rvs()
            inflow_time = float(sample_trip['inflowTime'])
            inflow_time = inflow_time/speed_factor
            departure_time = departure_times[i] + inflow_time
            departure_time = round(departure_time)

            

            sumo_trip = SUMOTrip(sample_trip, departure_time, i+total_trips_processed)
            sumo_trips.append(sumo_trip)

        return sumo_trips