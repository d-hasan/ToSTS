import os
import sys
import pdb


import networkx as nx
import matplotlib.pyplot as plt 
import matplotlib


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import sumolib


net_path = 'toronto-data/osm.net.xml'
net = sumolib.net.readNet(net_path)


def create_graph(net):
    graph = nx.DiGraph()

    sumo_nodes = net.getNodes()
    nodes = [node.getID() for node in sumo_nodes]

    graph.add_nodes_from(nodes)

    sumo_edges = net.getEdges()
    # edge = sumo_edges[0]
    # pdb.set_trace()
    edges = [(edge.getFromNode().getID(), edge.getToNode().getID(), {'weight':1/edge.getLength()}) for edge in sumo_edges]

    graph.add_edges_from(edges)

    return graph 



def plot_graph(graph):
    plt.clf()
    plt.cla()
    nodes = list(graph.nodes)
    degrees = dict(graph.degree)
    node_colors = [degrees[node] for node in nodes]

    edges = list(graph.edges(data='weight'))
    edge_colors = [edge[-1] for edge in edges]
    edges = [edge[:2] for edge in edges]

    # pdb.set_trace()
    # drawing nodes and edges separately so we can capture collection for colobar
    pos = nx.spring_layout(graph, weight='weight', iterations=100)
    ec = nx.draw_networkx_edges(graph, pos, edgelist=edges,)
    nc = nx.draw_networkx_nodes(graph, pos, nodelist=nodes, node_color=node_colors, 
                                with_labels=False, node_size=20, cmap=matplotlib.cm.jet)
    plt.colorbar(nc)
    # plt.colorbar(ec)
    plt.axis('off')
    plt.show()
