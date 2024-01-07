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
import HelperClass


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
    
    def _getNodeNamebasedOnCoordinates(self,coordinates):
        # Initialize node name
        nodeName = ""
        
        # Loop through all nodes and their attributes in the graph
        for node, attributes in self.graph.nodes(data=True):
            # Check if the current node has a "pos" attribute
            if "pos" in attributes:
                # Check if the coordinates match the "pos" attribute of the current node
                if coordinates == attributes["pos"]:
                    # Assign the name of the current node to nodeName and exit the loop
                    nodeName = node
                    break
        
        # Return the name of the node based on the provided coordinates
        return nodeName
    
    @IPPerfMonitor
    def _nearestInterim(self,currentNode,checkedInterimGoalList):
        
        # List 1: Coordinates
        # List 2: Distance
        # List 3: Name
        result_interim = [[],[],[]]
        
        # Loop over all checked interim goals
        i = 0
        for next_pos_node in checkedInterimGoalList:
            
            # Current position and position of the next interim goal
            point_current = (currentNode[0] , currentNode[1])
            point_pos_next = (next_pos_node[0],next_pos_node[1])

            # Add coordinates and distance to the result list
            result_interim[0].append(next_pos_node)
            result_interim[1].append(euclidean(point_current,point_pos_next))
            i += 1
            
       
        # Assign names to the nearest points
        for nearest in result_interim[0]:
            for node, attributes in self.graph.nodes(data=True):
                if "pos" in attributes:
                    if nearest == attributes["pos"]:
                        result_interim[2].append(node)
                        break
                        
        # Find the minimum distance and the index of the nearest interim goal
        minimum_value = min(result_interim[1])
        minimum_index = result_interim[1].index(minimum_value)
        
        # Return the information of the nearest interim goal
        return [result_interim[0][minimum_index], result_interim[1][minimum_index],result_interim[2][minimum_index]]
            
    
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
        
        # Add Goallist to InterimGoalList
        checkedInterimGoalList.append(checkedGoalList[0])

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
                 self.graph.add_node("start", pos=checkedStartList[0], color='lawngreen')
                 self.graph.add_edge("start", node[0])
                 break    

        # Iterate through each interim goal in the list
        for interimGoal in range(len(checkedInterimGoalList)):

            result = self._nearestNeighbours(checkedInterimGoalList[interimGoal],config["radius"])
            
            # Create a unique name for the current interim goal node
            nameOfNode = "interim" + str(interimGoal)

            for node in result:
                if not self._collisionChecker.lineInCollision(checkedInterimGoalList[interimGoal],node[1]['pos']):
                    self.graph.add_node(nameOfNode, pos=checkedInterimGoalList[interimGoal], color='Coral')
                    self.graph.add_edge(nameOfNode, node[0])
                    break

        result = self._nearestNeighbours(checkedGoalList[0], config["radius"])
        for node in result:
            if not self._collisionChecker.lineInCollision(checkedGoalList[0],node[1]['pos']):
                 self.graph.add_node("goal", pos=checkedGoalList[0], color='Coral')
                 self.graph.add_edge("goal", node[0])
                 break
        
        try:
            # Calculate shortest distance to nearest interim from start 
            result_interim = self._nearestInterim(checkedStartList[0], checkedInterimGoalList)
            print("Ziel Interim:" + str(result_interim))
            
            # Plan path from start to nearest interim
            try_path = nx.shortest_path(self.graph, "start", result_interim[2])
            print("Try Path: "+ str(try_path))

            # Initialize path and loop break condition
            path = list()
            breakcondition = False
            
            # Loop to iteratively plan a path through interim goals
            while not breakcondition:
                print("")
                print("While Schleife beginnt")
                    
                print("TRYPATH :",(try_path))
                
                # Iterate through steps in the current try_path
                for step in try_path:
                    print("")
                    print("For-Schleife beginnt")
                    print("Aktueller Node (step): ", step)
                    
                    # Add step to the final path
                    path.append(step)
                    HelperClass.HelperClass.printInColor("Aktueller Pfad: " + str(path), 'Dodgerblue')
                    
                    # Find nearest interim goal from the current step in Try-path
                    new_result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)
                    print("Nächstes Ziel-Interim: ", new_result_interim)               
                    
                    # Check if the distance to the new interim is zero (Interim is reached)
                    if new_result_interim[1] == 0.0:
                        print("Ziel-Interim erreicht")
                        
                        # Check if there is only one interim goal remaining, this means all interims are reached
                        if (len(checkedInterimGoalList) == 1 ):
                            
                            # End the loop
                            breakcondition = True
                            break
                        
                        # Remove the current interim goal from the list
                        else:
                        
                            checkedInterimGoalList.remove(result_interim[0])

                        # Calculate the shortest distance to the new interim goal
                        result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)
                        print("Neues Ziel-Interim: ", result_interim)
                        
                        # Get the node name of current step based on coordinates
                        nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])

                        # Plan a new path from the current position to the new interim goal
                        try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])

                        # Remove first step of Try-Path because it is already reached
                        try_path.pop(0)

                        print("Neuer Trypath: ", try_path)
                        break

                    if new_result_interim != result_interim:
                        
                        print("NewResultInterim !!!=== ResultInterim")
                        
                        result_interim = new_result_interim
                        
                        # Get the node name of current step based on coordinates
                        
                        nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])
                        
                        try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])

                        # Remove first step of Try-Path because it is already reached
                        try_path.pop(0)

                        print("Neuer Trypath: ", try_path)
                        break
            
            HelperClass.HelperClass.printInColor("Solution =  " + str(path), 'lawngreen')
            
            # # Calculate shortest distance to nearest interim from start 
            # result_interim = self._nearestInterim(checkedStartList[0], checkedInterimGoalList)
            # print("Result:" + str(result_interim))
            
            # # Plan path from start to nearest interim
            # try_path = nx.shortest_path(self.graph, "start", result_interim[2])
            # print("PFAD: "+ str(try_path)+ str(type(try_path)))
            
            # # Initialize path and loop break condition
            # path = list()
            # breakcondition = False
            
            # # Loop to iteratively plan a path through interim goals
            # while not breakcondition:
            # #print(breakcondition)
                    
            #     print("TRYPATH :" + str(try_path))
                
            #     # Iterate through steps in the current try_path
            #     for step in try_path:
            #         print("for-schleife beginnt")
                    
            #         # Find nearest interim goal from the current step in Try-path
            #         new_result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)
                    
            #         # print("new_result_interim"+ str(new_result_interim))
                    
            #         # Check if the new interim goal is the same as the previous one
            #         if new_result_interim[2] == result_interim[2]:
                        
            #             # Add step to the final path
            #             path.append(step)
                        
            #             print("aktuelles Interim" + str(result_interim))
            #             print("Abstand" + str(new_result_interim[1]))
                        
            #             # Check if the distance to the new interim is zero (Interim is reached)
            #             if new_result_interim[1] == 0.0:
            #                 print("DER ABSTAND IST NULL")
                            
            #                 # Get the node name of current step based on coordinates
            #                 nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])
            #                 print("Akutlele Interim Golalist: " +str(checkedInterimGoalList))
                            
            #                 # Check if there is only one interim goal remaining, this means all interims are reached
            #                 if (len(checkedInterimGoalList) == 1 ):
                                
            #                     # End the loop
            #                     breakcondition = True
            #                     break
                            
            #                 # Remove the current interim goal from the list
            #                 else:
            #                     checkedInterimGoalList.remove(result_interim[0])

            #                 # Calculate the shortest distance to the new interim goal
            #                 result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)
                            
            #                 # Plan a new path from the current position to the new interim goal
            #                 try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])
                            
            #                 break

            #         # If new interim goal is not the same as the current one  
            #         else:
            #             print("bin im Else")
                        
            #             # Update the current interim goal through the new interim goal information
            #             result_interim = new_result_interim
                        
            #             # Get the node name of current step based on coordinates
            #             nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])
                        
            #             # Plan a new try path from the current position to the new interim goal
            #             try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])

            #             break
               
            # print("Kompletter Pfad: "+ str(path))
        except Exception as e :
            print("Fehler " + str(e))
            return []
        
        # Return final path
        return path
    


        """
        # wie wolln wir vorgehen
        
        # schleife die schaut was von anderen Interims der nächste ist
        # kann interium Punkt 1 ohne Kollision zum nächsten
        # falls JA: -> fügs in die liste ein -> dann ist das der shortest Path
        # falls Nein: wiederhole das und machs beim näctsen
        # wenn kein Interium mit den Bedgingungen gefunden wird, verbinde Node mit nächstem Interim auch wenn der Kollidiert
        
        # Neue Idee:
        
        # 1. Ausgabe Pfad von Start bis nähestes Interim
        # nx.shortestPath (Start, neartestInterium)
        
        # 2. Wir nehmen dann die erste Node aus dem Pfad, nach Start (Node A)
        # 3. Dann rufen wir nearest Neighbour, wir wollen den Abstand von der aktuellen Node allen Interiums
        # 3b. Abgleichen mit Liste die schon abgehakt ist, wegschmeißen der Interium die schon verplant sind
        # 4. Interim mit dem kleinesten Abstand wird das nächste ZWischenziel -> Interim B
        # 5. nx.shorestPath (Node A, Interim B)
        # 6. gehe wieder zu 2
        # wenn wir durch alle Interiums durch sind (interimGoal = Liste aus 3b ), verbindung mit dem Ziel
        
        """