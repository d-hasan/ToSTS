from collections import defaultdict
import configparser
import json 
import os
import random 
import pdb 

import numpy as np
from numpy.core.defchararray import center
from numpy.lib.financial import irr 
import pandas as pd 
from scipy import signal 

import utils 


class TTSData():

    def __init__(self, config_path):
        super().__init__()
        
        self.config = utils.get_config(config_path)
        self.time_interval = int(self.config['DEFAULT']['time_interval'])
        self.filter_width = int(self.config['DEFAULT']['filter_width'])

        self.tts_path = self.config['DEFAULT']['tts_path']
        self.savedir = self.config['DEFAULT']['savedir']

        times, time_data_dict, time_data_sparse, origin_counts, destination_counts = self.parse_data(self.tts_path)
        self.times = times
        self.time_data_dict = time_data_dict
        self.time_data_sparse = time_data_sparse
        self.origin_counts = origin_counts
        self.destination_counts = destination_counts


        regular_times, regular_time_data_dict, regular_time_data_sparse = self.round_irregular_timings()
        self.regular_times = regular_times
        self.regular_time_data_dict = regular_time_data_dict
        self.regular_time_data_sparse = regular_time_data_sparse

        smooth_time_data_dict, smooth_time_data_sparse = self.smooth_trip_counts()
        self.smooth_time_data_dict = smooth_time_data_dict
        self.smooth_time_data_sparse = smooth_time_data_sparse

        self.save_data()

    
    def parse_data(self, filepath):
        with open(filepath,'r') as f:
            lines = f.readlines()

        lines = [line.strip() for line in lines if not line.isspace()]

        origin_counts = defaultdict(int)
        destination_counts = defaultdict(int)
        times = set()
        time_data_sparse = {}
        time_data_dict = {}
        curr_time = -1

        for line in lines:
            if 'TABLE' in line:
                curr_time = int(line.split('(')[-1][:-1])

                time_data_sparse[curr_time] = []
                time_data_dict[curr_time] = defaultdict(dict)

                times.add(curr_time)
            
            elif curr_time > 0:
                try:
                    data = [int(item) for item in line.split()]
                
                # make this explicit, in what it catches
                except ValueError:
                    continue 
                else:
                    origin = data[0]
                    destination = data[1]

                    origin_counts[origin] += 1
                    destination_counts[destination] += 1

                    time_data_sparse[curr_time].append(data)
                    time_data_dict[curr_time][origin][destination] = data[2]

        for time in times:
            time_data_sparse[time] = np.array(time_data_sparse[time])

        return times, time_data_dict, time_data_sparse, origin_counts, destination_counts


    def save_data(self):
        save_dict = {str(key): self.time_data_sparse[key] for key in self.time_data_sparse.keys()}
        save_dict['times'] = np.array(list(self.times))
        np.savez(os.path.join(self.savedir, 'tts.npz'), **save_dict)

        with open(os.path.join(self.savedir,' tts.json'), 'w') as f:
            json.dump(self.time_data_dict, f, indent=4)

        save_dict = {str(key): self.regular_time_data_sparse[key] for key in self.regular_time_data_sparse.keys()}
        save_dict['times'] = np.array(list(self.regular_times))
        np.savez(os.path.join(self.savedir, 'tts_regular_times.npz'), **save_dict)

        with open(os.path.join(self.savedir,' tts_regular_times.json'), 'w') as f:
            json.dump(self.regular_time_data_dict, f, indent=4)


        save_dict = {str(key): self.smooth_time_data_sparse[key] for key in self.smooth_time_data_sparse.keys()}
        save_dict['times'] = np.array(list(self.regular_times))
        np.savez(os.path.join(self.savedir, 'tts_smoothed_counts.npz'), **save_dict)

        # Can't save these without running into some weird error about int64 keys??

        # with open(os.path.join(self.savedir,' tts_smoothed_counts.json'), 'w') as f:
        #     json.dump(self.smooth_time_data_dict, f, indent=4)


    def round_irregular_timings(self):
        ''' There are ~21k out of 17.5mil trips that do not start at the usual 5 minute intervals.
            These trips are rounded to the nearest 5 minute interval.
        '''

        trip_times = sorted(list(self.times))
        max_trip_time = trip_times[-1]
        regular_trip_times = [trip_time for trip_time in trip_times if trip_time % 5 == 0 or trip_time == max_trip_time]
        irregular_trip_times = [trip_time for trip_time in trip_times if trip_time % 5 != 0 and trip_time != max_trip_time]

        regular_time_data_dict = {}
        for trip_time in regular_trip_times:
            regular_time_data_dict[trip_time] = defaultdict(dict)

            for origin in self.time_data_dict[trip_time]:
                regular_time_data_dict[trip_time][origin] = defaultdict(int)
                for destination in self.time_data_dict[trip_time][origin]:
                    trip_count = self.time_data_dict[trip_time][origin][destination] 
                    regular_time_data_dict[trip_time][origin][destination] = trip_count

        for irregular_time in irregular_trip_times:
            remainder = irregular_time % 5
            rounded_time = irregular_time - remainder if remainder < 2.5 else irregular_time + (5-remainder)
            if (rounded_time % 100) % 60 == 0:
                rounded_hour = rounded_time // 100
                rounded_time = (rounded_hour + 1) * 100

            rounded_time = min(rounded_time, max_trip_time)


            for origin in self.time_data_dict[irregular_time]:
                for destination in self.time_data_dict[irregular_time][origin]:
                    trip_count = self.time_data_dict[irregular_time][origin][destination] 
                    if origin in regular_time_data_dict[rounded_time]:
                        try:
                            regular_time_data_dict[rounded_time][origin][destination] += trip_count
                        except:
                            print('error')
                            import pdb; pdb.set_trace()
                    else:
                        regular_time_data_dict[rounded_time][origin] = defaultdict(int)
                        regular_time_data_dict[rounded_time][origin][destination] = trip_count

        regular_time_data_sparse = {}
        for trip_time in regular_time_data_dict:
            regular_time_data_sparse[trip_time] = []
            for origin in regular_time_data_dict[trip_time]:
                for destination in regular_time_data_dict[trip_time][origin]:
                    trip_count = regular_time_data_dict[trip_time][origin][destination]
                    data = [origin, destination, trip_count]
                    regular_time_data_sparse[trip_time].append(data)

        for trip_time in regular_time_data_sparse:
            regular_time_data_sparse[trip_time] = np.array(regular_time_data_sparse[trip_time])

        return regular_trip_times, regular_time_data_dict, regular_time_data_sparse


    def smooth_trip_counts(self):
        window = signal.triang(self.filter_width)

        trip_times = sorted(list(self.regular_times))
        trip_counts = []
        for trip_time in trip_times:
            trip_count = self.regular_time_data_sparse[trip_time][:,-1].sum()
            trip_counts.append(trip_count)
        trip_counts = np.array(trip_counts)

        filtered_counts = signal.convolve(trip_counts, window)/window.sum() 

        time_data_expanded = defaultdict(list)
        for trip_time in self.regular_time_data_sparse:
            trips = self.regular_time_data_sparse[trip_time]
            expanded_trips = []

            for trip in trips:
                origin, destination, trip_count = trip 
                orig_dest = [[origin, destination]]
                expanded_trips += orig_dest * trip_count 

            random.shuffle(expanded_trips)
            
            time_data_expanded[trip_time] = expanded_trips

        weighted_allocations = defaultdict(list)
        for trip_time in time_data_expanded:
            trips = time_data_expanded[trip_time]
            total_trips = len(trips)

            allocation_counts = (window * total_trips)/window.sum()
            allocation_counts = np.round(allocation_counts).astype(int)
            allocation_index = [0] + list(allocation_counts.cumsum())

            for start_index, end_index in zip(allocation_index[:-1], allocation_index[1:]):
                allocation = trips[start_index:end_index]
                weighted_allocations[trip_time].append(allocation)

        smooth_trip_data = defaultdict(list)
        for trip_time in weighted_allocations:
            center_index = self.filter_width // 2

            for i, trips in enumerate(weighted_allocations[trip_time]):
                trip_time_minutes = utils.convert_clock_to_minutes(trip_time)
                smooth_time_minutes = trip_time_minutes + (i - center_index) * self.time_interval
                smooth_time = utils.convert_minutes_to_clock(smooth_time_minutes)
                smooth_trip_data[smooth_time] += trips 

        smooth_time_data_dict = {}
        smooth_time_data_sparse = defaultdict(list)
        for trip_time in smooth_trip_data:
            smooth_time_data_dict[trip_time] = defaultdict(dict)
            trip_dict = defaultdict(int)

            for trip in smooth_trip_data[trip_time]:
                origin, destination = trip 

                if origin not in smooth_time_data_dict[trip_time]:
                    smooth_time_data_dict[trip_time][origin] = defaultdict(int)

                smooth_time_data_dict[trip_time][origin][destination] += 1
                
                trip_dict[(origin, destination)] += 1
            
            for trip in trip_dict:
                origin, destination = trip
                trip_count = trip_dict[trip]
                trip_data = [origin, destination, trip_count]
                smooth_time_data_sparse[trip_time].append(trip_data)

        for trip_time in smooth_time_data_sparse:
            smooth_time_data_sparse[trip_time] = np.array(smooth_time_data_sparse[trip_time])

        return smooth_time_data_dict, smooth_time_data_sparse

