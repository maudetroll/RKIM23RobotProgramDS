# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import IPPRMBase
from IPPerfMonitor import IPPerfMonitor
import networkx as nx
import random
import numpy as np
import math

# reduce coding effort by using function provided by scipy
from scipy.spatial.distance import euclidean, cityblock

class BasicPRM(IPPRMBase.PRMBase):

    def __init__(self, _collChecker):
        super(BasicPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.reachedInterims = [] 

    
    @IPPerfMonitor
    def _inSameConnectedComponent(self, node1, node2):
        """ Check whether to nodes are part of the same connected component using
            functionality from NetworkX
        """
        for connectedComponent in nx.connected_components(self.graph):
            if (node1 in connectedComponent) & (node2 in connectedComponent):
                return True

        return False

    
    @IPPerfMonitor
    def _nearestNeighbours(self, pos, radius):
        """ Brute Force method to find all nodes of a 
        graph near the given position **pos** with in the distance of
        **radius** """

        result = list()

        for node in self.graph.nodes(data=True):
            if euclidean(node[1]['pos'],pos) <= radius:
                result.append(node)
                
        return result
    @IPPerfMonitor
    def _nearestInterim(self,checkedStartList,checkedInterimGoalList,checkedGoalList,pos):
        
        result_interim = list()
#        for node in self.graph.nodes(data=True):              
#            if 'interim' in str(node[0]):
#                result_interim.append(node)
                #
#            print(str(euclidean(node[1]['pos'],pos)))
        #for node in checkedInterimGoalList:
        #    euclidean(node[1]['pos'],pos)
        
        print(str(checkedInterimGoalList))
        
        # wie wolln wir vorgehen
        
        # schleife die schaut was von anderen Interims der nächste ist
        # kann interium Punkt 1 ohne Kollision zum nächsten
        # falls JA: -> fügs in die liste ein -> dann ist das der shortest Path
        # falls Nein: wiederhole das und machs beim näctsen
        # wenn kein Interium mit den Bedgingungen gefunden wird, verbinde Node mit nächstem Interim auch wenn der Kollidiert
        
        # Neue Idee:
        
        # 1. Ausgabe Pfad von Start bis nähestes Interim
        # 2. Wir nehmen dann die erste Node aus dem Pfad, nach Start (Node A)
        # 3. Dann rufen wir nearest Neighbour, wir wollen den Abstand von der aktuellen Node allen Interiums
        # 3b. Abgleichen mit Liste die schon abgehakt ist, wegschmeißen der Interium die schon verplant sind
        # 4. Interim mit dem kleinesten Abstand wird das nächste ZWischenziel -> Interim B
        # 5. nx.shorestPath (Node A, Interim B)
        # 6. gehe wieder zu 2


                
        return result_interim
    
    
    @IPPerfMonitor
    def _learnRoadmapNearestNeighbour(self, radius, numNodes):
        """ Generate a roadmap by given number of nodes and radius, that should be tested for connection."""
        # nodeID is used for uniquely enumerating all nodes and as their name
        nodeID = 1
        while nodeID <= numNodes:
        
            # Generate a 'randomly chosen, free configuration'
            newNodePos = self._getRandomFreePosition()
            self.graph.add_node(nodeID, pos=newNodePos)
            
            # Find set of candidates to connect to sorted by distance
            result = self._nearestNeighbours(newNodePos, radius)

            # for all nearest neighbours check whether a connection is possible
            for data in result:
                if self._inSameConnectedComponent(nodeID,data[0]):
                    continue
                
                if not self._collisionChecker.lineInCollision(newNodePos,data[1]['pos']):
                    self.graph.add_edge(nodeID,data[0])
            
            nodeID += 1
    
    @IPPerfMonitor
    def planRoundPath(self, startList,interimGoalList, goalList, config):
        """
        
        Args:
            start (array): start position in planning space
            goal (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["radius"]   = 5.0
            config["numNodes"] = 300
            config["useKDTree"] = True
            
            startList = [[1,1]]
            goalList  = [[10,1]]
            
            instance.planPath(startList,goalList,config)
        
        """
        # 0. reset
        self.graph.clear()
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedInterimGoalList, checkedGoalList = self._checkStartGoal(startList,interimGoalList, goalList)
        
        #print(checkedStartList)
        #print(checkedInterimGoalList)
        #print(checkedGoalList)

        # 2. learn Roadmap
        self._learnRoadmapNearestNeighbour(config["radius"],config["numNodes"])

        # 3. find connection of start and goal to roadmap
        # find nearest, collision-free connection between node on graph and start
        result = self._nearestNeighbours(checkedStartList[0], config["radius"])
        for node in result:
            if not self._collisionChecker.lineInCollision(checkedStartList[0],node[1]['pos']):
                 self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                 self.graph.add_edge("start", node[0])
                 break
        #print("InteriumListe:" + str(result_interim))
        
        


        for interimGoal in range(len(checkedInterimGoalList)):
            print("InterimGoal: " + str(interimGoal))
            print("Was steht in der Liste: " + str(checkedInterimGoalList[interimGoal]))
            
            result = self._nearestNeighbours(checkedInterimGoalList[interimGoal],config["radius"])
            result_interim = self._nearestInterim(checkedInterimGoalList[interimGoal])
            nameOfNode = "interim" + str(interimGoal)

            for node in result:
                if not self._collisionChecker.lineInCollision(checkedInterimGoalList[interimGoal],node[1]['pos']):
                    self.graph.add_node(nameOfNode, pos=checkedInterimGoalList[interimGoal], color='lightgreen')
                    self.graph.add_edge(nameOfNode, node[0])
                    break

        result = self._nearestNeighbours(checkedGoalList[0], config["radius"])
        for node in result:
            if not self._collisionChecker.lineInCollision(checkedGoalList[0],node[1]['pos']):
                 self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
                 self.graph.add_edge("goal", node[0])
                 break
        
        try:
            
            interim_count = len(checkedInterimGoalList)

            # Calc shortest Path To Interium 
            
            # Connect Start with first interim
            path = nx.shortest_path(self.graph, "start", "interim0")

            # Connect all interims
            for i in range(interim_count - 1):
                interim_name_current = "interim" + str(i)
                interim_name_next = "interim" + str(i + 1)
                
                path += nx.shortest_path(self.graph, interim_name_current, interim_name_next)
                
            # Connect last interim with goal
            path += nx.shortest_path(self.graph, "interim" + str(interim_count - 1), "goal")

            print(path)
        except:
            return []
        return path
    
    
   # def shortenPathToInterium(self,result):
        # finde nearste Neighbour zu aktueller Node
        #nextinterim = self.graph.node
        
        # nearestinterim = Interimsname (e.g. interium2) and return object
       # nearestinterim
        
      #  nextnode =
        
      #  return nearestinterim