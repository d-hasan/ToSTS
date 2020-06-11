from collections import defaultdict
import json 
import os

import numpy as np 
import pandas as pd 

filepath = 'tts_data/tts_alltrips.txt'


class TTSData():

    def __init__(self, tts_path, savedir):
        super().__init__()

        self.tts_path = tts_path
        self.savedir = savedir

        times, time_data_dict, time_data_sparse, origin_counts, destination_counts = self.parse_data(self.tts_path)

        self.times = times
        self.time_data_dict = time_data_dict
        self.time_data_sparse = time_data_sparse
        self.origin_counts = origin_counts
        self.destination_counts = destination_counts

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

    

# if __name__ == '__main__':
#     times, time_data_dict, time_data_sparse, origins, destinations = parse_data(filepath)
#     save_data(times, time_data_dict, time_data_sparse)
