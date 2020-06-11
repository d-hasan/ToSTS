import networkx as nx 

from region_graph import RegionGraph

class TripPlanner():
    
    def __init__(self, sim_net, tts_trips, traffic_regions):
        super().__init__()
        self.sim_net = sim_net
        self.trips = tts_trips
        self.traffic_regions = traffic_regions
        self.region_graph = RegionGraph(self.traffic_regions.region_gdf, self.traffic_regions.hways_gs)
        self.aoi_taz = self.get_aoi_taz()
        
    
    def get_aoi_taz(self):
        taz_gdf = self.traffic_regions.taz_gdf
        intersecting_index = [self.sim_net.boundary_polygon.intersects(taz) for taz in taz_gdf.geometry]
        taz_aoi = taz_gdf[intersecting_index]
        taz_aoi = set(list(taz_aoi.GTA06))
        return taz_aoi
    
        
    def generate_trips(self):
        for time in self.trips:
            trips = self.trips[time]
            for trip in trips:
                trip_type = self.classify_trip(trips)
        
                if trip_type == 0b00:
                    sumo_trip = self.trip_ext_to_ext(trip)
                elif trip_type == 0b01:
                    sumo_trip = self.trip_ext_to_int(trip)
                elif trip_type == 0b10:
                    sumo_trip = self.trip_int_to_ext(trip)
                elif trip_type == 0b11:
                    sumo_trip = self.trip_int_to_int(trip)

                    
    def classify_trip(self, od_data):
        trip_type = 0b00
        origin = od_data[0]
        destination = od_data[1]
        if origin in self.aoi_taz:
            trip_type = trip_type | 0b10
        if destination in self.aoi_taz:
            trip_type = trip_type | 0b01
        return trip_type
    
    
    def trip_ext_to_ext(self, trip):
        return None
    
    def trip_ext_to_int(self, trip):
        return None 
    
    def trip_int_to_ext(self, trip):
        return None 
    
    def trip_int_to_int(self, trip):
        return None 