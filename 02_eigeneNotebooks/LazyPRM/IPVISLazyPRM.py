# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import networkx as nx
from HelperPackage import HelperClass

def lazyPRMVisualize(planner, solution = [] , ax=None, nodeSize = 300):
    
    graph = planner.graph.copy()
    collChecker = planner._collisionChecker
    collEdges = planner.collidingEdges
    nonCollEdges = planner.nonCollidingEdges
    
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    pos = nx.get_node_attributes(graph,'pos')
    color = nx.get_node_attributes(graph,'color')

    if collEdges != []:
        collGraph = nx.Graph()
        collGraph.add_nodes_from(graph.nodes(data=True))

        for i in collEdges:
            collGraph.add_edge(i[0],i[1])
            
        nx.draw_networkx_edges(collGraph,pos,alpha=0.5,edge_color='r',width=5)
    
    nx.draw(graph, pos = pos, node_color = list(color.values()))
    
    # draw all connected components, emphasize the largest one
    Gcc=(graph.subgraph(c) for c in nx.connected_components(graph))
    G0=next(Gcc) 
    
    # how largest connected component
    nx.draw_networkx_edges(G0,pos,
                               edge_color='b',
                               width=3.0, style='dashed',
                               alpha=0.5,
                            )
    
    if nonCollEdges != []:
        nonCollGraph = nx.Graph()
        nonCollGraph.add_nodes_from(graph.nodes(data=True))

        for i in nonCollEdges:
            nonCollGraph.add_edge(i[0],i[1])
        nx.draw_networkx_edges(nonCollGraph,pos,alpha=0.8,edge_color='yellow',width=5)

    # show other connected components
    for Gi in Gcc:
        
        if len(Gi) >1:
            nx.draw_networkx_edges(Gi,pos,edge_color='b',alpha=0.1, width=1.0)

    collChecker.drawObstacles(ax)
    
    # draw start
    if "start" in graph.nodes(): 
        nx.draw_networkx_nodes(graph,pos,nodelist=["start"],
                                   node_size=300,
                                   node_color='lawngreen',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"start": "S"},  ax = ax)

    nodesToString= str(planner.graph.nodes())
    amountIterims = nodesToString.count('interim')
    
    # loop to visualize the interims in the plot
    i = 0
    for interim in range(amountIterims):
        name = "interim" + str(i)
        nx.draw_networkx_nodes(graph,pos,nodelist=[name],
                                    node_size=300,
                                    node_color='Gold',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={name: "I"},  ax = ax)
        i += 1


    if solution != []:
        
        # Create a new graph for the solution
        solGraph = nx.Graph()

        # Iterate over the elements in the solution list to create edges between consecutive elements      
        for i in range(len(solution) - 1):
            current_element = solution[i]
            next_element = solution[i + 1]
            solGraph.add_edge(current_element,next_element)
        
        # Create a subgraph of solGraph using the nodes in the solution list
        Gsp = nx.subgraph(solGraph,solution)

        # Draw the edges of the subgraph
        nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=10)
    
    return