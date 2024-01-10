# coding: utf-8

"""
This code is part of the course 'Innovative Programmiermethoden für Industrieroboter' (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import networkx as nx

def visibilityPRMVisualize(planner, solution, ax = None, nodeSize = 300):
    print("Visualsierung beginnt")
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    graph = planner.graph
    statsHandler = planner.statsHandler
    collChecker = planner._collisionChecker
    pos = nx.get_node_attributes(graph,'pos')
    color = nx.get_node_attributes(graph,'color')

    if statsHandler:
        print(statsHandler.graph.nodes())
        print("START ", statsHandler.graph.nodes()['start'])
        print(statsHandler.graph.nodes()[286])

        
        statPos = nx.get_node_attributes(statsHandler.graph,'pos')
        nx.draw(statsHandler.graph, pos=statPos, alpha=0.2,edge_color='y',node_size=nodeSize)

    # draw graph (nodes colorized by degree)
    nx.draw(graph, pos = pos, nodelist=color.keys(), node_color = color.values(), ax=ax)   
    nx.draw_networkx_edges(graph,pos,
                               edge_color='r',
                               width=3.0, ax=ax
                            )
    print("ZEile 30")
    collChecker.drawObstacles(ax)
    # get nodes based on solution path
    Gsp = nx.subgraph(graph,solution)

    # draw edges based on solution path
    nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=10, label="Solution Path",ax=ax)
        
    # draw start and goal
    # draw start and goal
    print("Start beginnt")
    if "start" in graph.nodes(): 
        nx.draw_networkx_nodes(graph,pos,nodelist=["start"],
                                   node_size=nodeSize,
                                   node_color='lawngreen',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"start": "S"},  ax = ax)
        

    # Count the amount of interims in Node of the main graph
    nodesToString= str(planner.graph.nodes())
    amountIterims = nodesToString.count('interim')
    
    # loop to visualize the interims in the plot
    i = 0
    for interim in range(amountIterims):
        name = "interim" + str(i)
        nx.draw_networkx_nodes(graph,pos,nodelist=[name],
                                    node_size=nodeSize,
                                    node_color='Dodgerblue',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={name: "I"},  ax = ax)
        i += 1

'''

    if "goal" in graph.nodes():
        nx.draw_networkx_nodes(graph,pos,nodelist=["goal"],
                                   node_size=nodeSize,
                                   node_color='#DD0000',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={"goal": "G"},  ax = ax)

'''