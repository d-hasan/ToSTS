import networkx as nx 

class RegionGraph():
    
    def __init__(self, regions, hways):
        self.hway_graph = nx.MultiGraph()
        self.trivial_graph = nx.Graph()
        self.add_nodes(regions)
        self.add_trivial_edges(regions)
        self.add_hway_edges(regions, hways)
        
        
    def add_nodes(self, regions):
        ''' Adds all planning districts as nodes.
        '''
        for index, _ in regions.iterrows():
            self.hway_graph.add_node(index)
            self.trivial_graph.add_node(index)

            
    def add_trivial_edges(self, regions):
        ''' Adds trivial edges for planning districts that are adjacent.
        '''
        nodes = set(self.trivial_graph.nodes)
        for node in nodes:
            other_nodes = nodes.difference({node})
            region = regions.loc[node].geometry
            edges = []
            for other_node in other_nodes:
                other_region = regions.loc[other_node].geometry
                if region.intersects(other_region):
                    edges.append((node, other_node))
            self.trivial_graph.add_edges_from(edges, weight=1)
            
    def add_hway_edges(self, regions, hways):
        ''' Adds edges for highway connections between regions.
        '''
        nodes = set(self.hway_graph.nodes)
        for node in nodes:
            region = regions.loc[node].geometry
            neighbours = self.trivial_graph.neighbors(node)
            for neighbour in neighbours:
                neighbour_region = regions.loc[neighbour].geometry
                for name, hway in hways.iteritems():
                    if hway.intersects(region) and hway.intersects(neighbour_region):
                        edge_key = '{}-{}-{}'.format(min(node, neighbour), max(node, neighbour), name)
                        self.hway_graph.add_edge(node, neighbour, key=edge_key, hway=name, weight=1)   