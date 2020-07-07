import configparser

import geopandas as gpd 
import pandas as pd 
import matplotlib.pyplot as plt 
import pyproj
import shapely
from shapely.ops import unary_union
 
import utils 

class TrafficRegions():
    region_lookup = {
        0: 'N/A',
        1: 'Toronto',
        2: 'Durham',
        3: 'York',
        4: 'Peel',
        5: 'Halton',
        6: 'Hamilton',
        11: 'Niagara',
        12: 'Waterloo',
        13: 'Guelph',
        14: 'Wellington',
        15: 'Orangeville',
        16: 'Barrie',
        17: 'Simcoe',
        18: 'City of Kawartha Lakes',
        19: 'City of Peterborough',
        20: 'Peterborough County',
        21: 'Orillia',
        22: 'Dufferin',
        23: 'Brantford',
        24: 'Brant',
        88: 'NUL', 
        98: 'External',
        99: 'Unknown'
    }
    
    def __init__(self, config_path, crs, location_offset):
        super().__init__()

        self.config = utils.get_config(config_path)

        self.taz_shape_path = self.config['DEFAULT']['taz_shape_path']
        self.pd_shape_path = self.config['DEFAULT']['pd_shape_path']
        self.orn_path = self.config['DEFAULT']['orn_path']

        self.crs = crs 
        self.location_offset = location_offset
        
        taz_gdf = self.load_taz_gdf(self.taz_shape_path)
        self.pd_gdf = self.load_pd_gdf(self.pd_shape_path)
        self.taz_gdf = self.assign_pd_for_taz(taz_gdf, self.pd_gdf)

        self.region_gdf = self.generate_region_gdf()
        self.hways_gs = self.load_highway_gdf(self.orn_path)
        

    def load_gdf(self, shape_path):
        gdf = gpd.read_file(shape_path)
        gdf = self.project_and_translate(gdf)
        return gdf 

    def load_taz_gdf(self, shape_path):
        gdf = self.load_gdf(shape_path)
        gdf = gdf.set_index('GTA06')
        return gdf 

    def assign_pd_for_taz(self, taz_gdf, pd_gdf):
        no_taz_gdf = taz_gdf[taz_gdf.PD == 0]
        not_found = []
        for taz_index, taz_row in no_taz_gdf.iterrows():
            taz_polygon = taz_row.geometry
            taz_point = taz_polygon.representative_point()
            
            found_pd = False
            for pd_index, pd_row in pd_gdf.iterrows():
                pd_polygon = pd_row.geometry.buffer(0)
                if taz_point.within(pd_polygon):
                    taz_gdf.loc[taz_index, 'PD'] = pd_index
                    found_pd = True
                    break
            
            if not found_pd:
                # There's one polygon that somehow isn't assigned a PD number...
                # It's in York Region, North of PD 25
                not_found.append(taz_index)
                print(taz_index)
                overlapping_pd = []
                for pd_index, pd_row in pd_gdf.iterrows():
                    if taz_row.geometry.overlaps(pd_row.geometry):
                        print('\t', pd_index)
                        overlapping_pd.append([pd_index, pd_row])
                        
                if len(overlapping_pd) > 1:
                    # The TAZ boundary touches/intersects more than one PD polygon
                    distances = []
                    for pd_index, pd_row in overlapping_pd:
                        taz_centroid = taz_row.geometry.centroid
                        distance = taz_centroid.distance(pd_row.geometry.exterior)
                        distances.append(distance)

                    closest = sorted(zip(distances, overlapping_pd))[0][1]

                else:
                    closest = overlapping_pd[0]

                print('\tclosest:', closest[1][0])
                taz_gdf.loc[taz_index, 'PD'] = closest[0]
        return taz_gdf
        

    def load_pd_gdf(self, shape_path):
        gdf = self.load_gdf(shape_path)
        gdf = gdf.set_index('PD')
        return gdf 

    def load_highway_gdf(self, orn_path):
        print('Loading Ontario Road Network and extracting highways, this can take a few minutes.')
        orn_gdf = gpd.read_file(orn_path)
        hway_gdf =  orn_gdf[orn_gdf.ROAD_CLASS.isin(['Freeway', 'Expressway / Highway'])]
        hway_gdf = self.project_and_translate(hway_gdf)

        hways = {}
        for name, group in hway_gdf.groupby('ROUTE_NUMB'):
            hways[name] = unary_union(group.geometry.values)
        
        null_route = hway_gdf[hway_gdf.ROUTE_NUMB.isna()]
        for name, group in null_route.groupby('OFFICIAL_S'):
            if name in hways:
                hways[name] = unary_union(list(group.geometry.values) + [hways[name]])
            else:
                hways[name] = unary_union(group.geometry.values)


        hway_gs = gpd.GeoSeries(hways)

        return hway_gs

    # use the TAZ GDF to group regions and generate region polygons instead of PD GDF
    def generate_region_gdf(self):
        geometries = []
        region_shapes = {}
        for region_num, group in self.pd_gdf.groupby('REGION'):
            region_shapes[region_num] = self.region_lookup[region_num]
            geometries.append(unary_union(group.geometry.values))
            
        df = pd.DataFrame.from_dict(region_shapes, orient='index', columns=['NAME'])
        region_gdf = gpd.GeoDataFrame(df, geometry=geometries)
        return region_gdf

    def project_and_translate(self, gdf):
        gdf = gdf.to_crs(epsg=self.crs.to_epsg())
        x_off, y_off = self.location_offset
        gdf.geometry = gdf.geometry.affine_transform([1, 0, 0, 1, x_off, y_off])
        return gdf


    def plot_area(self, gdf, ax=None):
        ax.set_facecolor('xkcd:light blue')
        ax = gdf.plot(ax=ax, color='k', edgecolor='w', linewidth=3)
        return ax

    def plot_highways(self, ax=None):
        self.hways_gs.plot(ax=ax, cmap=plt.get_cmap('tab10'))
    
 