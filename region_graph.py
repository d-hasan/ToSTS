import time 

import networkx as nx 
import shapely 

class RegionGraph():
    
    def __init__(self, pd_gdf, region_gdf, hways):
        self.pd_graph = nx.MultiDiGraph()
        self.region_graph = nx.MultiDiGraph()
        self.add_nodes(pd_gdf, region_gdf)
        self.add_trivial_edges(pd_gdf, region_gdf)
        self.add_hway_edges(pd_gdf, region_gdf, hways)
        self.pd_paths = dict(nx.all_pairs_dijkstra_path(self.pd_graph))
        
        
    def add_nodes(self, pd_gdf, region_gdf):
        ''' Adds all planning districts as nodes.
        '''
        for index, _ in pd_gdf.iterrows():
            self.pd_graph.add_node(index)

        for index, _ in region_gdf.iterrows():
            self.region_graph.add_node(index)

            
    def add_trivial_edges(self, pd_gdf, region_gdf):
        ''' Adds trivial edges for planning districts that are adjacent.
        '''
        start_time = time.time()
        print('Computing Region Graph...')
        region_nodes = set(self.region_graph.nodes)
        for i, node in enumerate(region_nodes, 1):
            other_nodes = region_nodes.difference({node})
            region = region_gdf.loc[node].geometry
            for other_node in other_nodes:
                other_region = region_gdf.loc[other_node].geometry
                if region.intersects(other_region):
                    edge_key = '{}-{}-{}'.format(node, other_node, 'trivial')
                    self.region_graph.add_edge(node, other_node, key=edge_key, hway='trivial', weight=2)
            
            # print('\tCompleted {} out of {} regions in {:.1f}s'.format(i, len(region_nodes), time.time()-start_time))
        print('Completed Region Graph.')


        start_time = time.time()
        print('Computing Planning District Graph...')
        pd_nodes = set(self.pd_graph.nodes)
        for i, node in enumerate(pd_nodes, 1):
            other_nodes = pd_nodes.difference({node})
            plan_dist = pd_gdf.loc[node].geometry.buffer(0)
            for other_node in other_nodes:
                other_plan_dist = pd_gdf.loc[other_node].geometry.buffer(0)

                if plan_dist.intersects(other_plan_dist):
                    weight = plan_dist.centroid.distance(other_plan_dist.centroid)
                    edge_key = '{}-{}-{}'.format(node, other_node, 'trivial')
                    self.pd_graph.add_edge(node, other_node, key=edge_key, hway='trivial', weight=2.5)
            if i % 25 == 0:
                elapsed_time = time.time() - start_time
                eta = elapsed_time/i * (len(pd_nodes) - i)
                print('\tCompleted {} out of {} planning districts in {:.1f}s, ETA={:.1f}s'.format(i, len(pd_nodes), elapsed_time, eta))

            

    def add_hway_edges(self, pd_gdf, region_gdf, hways):
        ''' Adds edges for highway connections between regions.
        '''
        print('Computing Highway Edges for Region Graph...')
        region_nodes = set(self.region_graph.nodes)
        for node in region_nodes:
            region = region_gdf.loc[node].geometry
            neighbours = self.region_graph.neighbors(node)
            for neighbour in neighbours:
                neighbour_region = region_gdf.loc[neighbour].geometry
                for name, hway in hways.iteritems():
                    if hway.intersects(region) and hway.intersects(neighbour_region):
                        edge_key = '{}-{}-{}'.format(min(node, neighbour), max(node, neighbour), name)
                        self.region_graph.add_edge(node, neighbour, key=edge_key, hway=name, weight=1)  
        print('Completed Highway Edges for Region Graph.')


        print('Computing Highway Edges for Planning District Graph...')
        pd_nodes = set(self.pd_graph.nodes)
        for node in pd_nodes:
            plan_dist = pd_gdf.loc[node].geometry.buffer(0)
            neighbours = self.pd_graph.neighbors(node)
            for neighbour in neighbours:
                neighbour_plan_dist = pd_gdf.loc[neighbour].geometry.buffer(0)
                for name, hway in hways.iteritems():
                    if hway.intersects(plan_dist) and hway.intersects(neighbour_plan_dist):
                        weight = hway.intersection(plan_dist).length/2 + hway.intersection(neighbour_plan_dist).length/2
                        weight = weight/2 
                        edge_key = '{}-{}-{}'.format(min(node, neighbour), max(node, neighbour), name)
                        self.pd_graph.add_edge(node, neighbour, key=edge_key, hway=name, weight=1) 
        print('Completed Highway Edges for Planning District Graph.')

    

        