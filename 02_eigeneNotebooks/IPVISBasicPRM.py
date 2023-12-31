# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import matplotlib.pyplot as plt
import networkx as nx


def basicPRMVisualize(planner, solution, ax = None, nodeSize = 300):
    """ Draw graph, obstacles and solution in a axis environment of matplotib.
    """
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    graph = planner.graph
    collChecker = planner._collisionChecker
    
    pos = nx.get_node_attributes(graph,'pos')
    
    # draw graph (nodes colorized by degree)
    nx.draw_networkx_nodes(graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=nodeSize)
    nx.draw_networkx_edges(graph,pos,
                                ax = ax
                                 )
    Gcc = sorted(nx.connected_components(graph), key=len, reverse=True)
    G0=graph.subgraph(Gcc[0])# = largest connected component

    # how largest connected component
    nx.draw_networkx_edges(G0,pos,
                               edge_color='b',
                               width=3.0, ax = ax
                            )

    collChecker.drawObstacles(ax)
    
    
    # draw nodes based on solution path
    Gsp = nx.subgraph(graph,solution)
    nx.draw_networkx_nodes(Gsp,pos,
                            node_size=300,
                             node_color='g',  ax = ax)
        
    # draw edges based on solution path
    nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=10,  ax = ax)
        
    # draw start and goal
    if "start" in graph.nodes(): 
        nx.draw_networkx_nodes(graph,pos,nodelist=["start"],
                                   node_size=300,
                                   node_color='lawngreen',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"start": "S"},  ax = ax)


    #if "interim" in graph.nodes():
    #    nx.draw_networkx_nodes(graph,pos,nodelist=["interim"],
    #                               node_size=300,
    #                               node_color='#DD00DA',  ax = ax)
    #    nx.draw_networkx_labels(graph,pos,labels={"interim": "I"},  ax = ax)
        
                        
    # Count the amount of interims in Node of the main graph
    nodesToString= str(planner.graph.nodes())
    amountIterims = nodesToString.count('interim')
    
    # loop to visualize the interims in the plot
    i = 0
    for interim in range(amountIterims):
        name = "interim" + str(i)
        nx.draw_networkx_nodes(graph,pos,nodelist=[name],
                                    node_size= 300,
                                    node_color='Coral',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={name: "I"},  ax = ax)
        i += 1


    # if "goal" in graph.nodes():
    #     nx.draw_networkx_nodes(graph,pos,nodelist=["goal"],
    #                                node_size=300,
    #                                node_color='Dodgerblue',  ax = ax)
    #     nx.draw_networkx_labels(graph,pos,labels={"goal": "G"},  ax = ax)

