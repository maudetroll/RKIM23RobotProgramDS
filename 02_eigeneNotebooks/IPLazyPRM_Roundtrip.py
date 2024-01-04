# coding: utf-8
"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPRMBase import PRMBase
from scipy.spatial import cKDTree
import networkx as nx
import random
from scipy.spatial.distance import euclidean
import HelperClass

from IPPerfMonitor import IPPerfMonitor

class LazyPRM(PRMBase):

    def __init__(self, _collChecker):
        super(LazyPRM, self).__init__(_collChecker)
        
        self.graph = nx.Graph()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []
        self.nonCollidingEdges =[]

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
    def _buildRoadmap(self, numNodes, kNearest):
        
        # generate #numNodes nodes
        addedNodes = []
        for i in range(numNodes):
            pos = self._getRandomPosition()
            self.graph.add_node(self.lastGeneratedNodeNumber, pos=pos)
            addedNodes.append(self.lastGeneratedNodeNumber)
            self.lastGeneratedNodeNumber += 1


        # for every node in graph find nearest neigbhours
        posList = list(nx.get_node_attributes(self.graph,'pos').values())
        # print("poslist")
        # print(posList)
        kdTree = cKDTree(posList)
        
        # to see when _buildRoadmap has to be called again
        for node in addedNodes:

            # Find set of candidates to connect to sorted by distance
            result = kdTree.query(self.graph.nodes[node]['pos'],k=kNearest)
            for data in result[1]:
                c_node = [x for x, y in self.graph.nodes(data=True) if (y['pos']==posList[data])][0]
                if node!=c_node:
                    if (node, c_node) not in self.collidingEdges:
                        self.graph.add_edge(node,c_node)
                    else:
                        continue
    
    @IPPerfMonitor
    def _checkForCollisionAndUpdate(self,long_path):
        path = long_path[:2]
        print("Path", path)
        # first check all nodes
        for nodeNumber in path:
            if self._collisionChecker.pointInCollision(self.graph.nodes[nodeNumber]['pos']):
                print("Removed nodeNumber: "+ str(nodeNumber))
                self.graph.remove_node(nodeNumber)
                
                return True
        
        # check all path segments
              
        for elem in zip(path,path[1:]):
            #print elem

            print("*******")
            x = elem[0]
            y = elem[1]
            print("x", x)
            print("y",y)
            if self._collisionChecker.lineInCollision(self.graph.nodes()[x]['pos'],self.graph.nodes()[y]['pos']):
                self.graph.remove_edge(x,y)
                self.collidingEdges.append((x,y))
                # print("Colliding Edges: "+ str(self.collidingEdges))
                return True
            else:
                # Verhindern damit Interim nicht mit sich selbst verbindet
            #    if x != y:
                self.nonCollidingEdges.append((x,y))
                    # print("NONColliding Edges: "+ str(self.nonCollidingEdges))

                                                                                          
            
        return False
    
        
    @IPPerfMonitor
    def _checkForCollisionAndUpdate_MS(self,path):
        # first check all nodes
        for nodeNumber in path:
            if self._collisionChecker.pointInCollision(self.graph.nodes[nodeNumber]['pos']):
                print("Removed nodeNumber: "+ str(nodeNumber))
                self.graph.remove_node(nodeNumber)
                
                return True
        
        # check all path segments
              
        for elem in zip(path,path[1:]):
            #print elem
            x = elem[0]
            y = elem[1]
            if self._collisionChecker.lineInCollision(self.graph.nodes()[x]['pos'],self.graph.nodes()[y]['pos']):
                self.graph.remove_edge(x,y)
                self.collidingEdges.append((x,y))
                # print("Colliding Edges: "+ str(self.collidingEdges))
                return True
            else:
                # Verhindern damit Interim nicht mit sich selbst verbindet
            #    if x != y:
                self.nonCollidingEdges.append((x,y))
                    # print("NONColliding Edges: "+ str(self.nonCollidingEdges))

                                                                                          
            
        return False
        
    @IPPerfMonitor   
    def planRoundPath(self, startList, interimGoalList, goalList, config):
        """
        
        Args:
            startList (array): start position in planning space
            goalList (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["initialRoadmapSize"] = 40 # number of nodes of first roadmap
            config["updateRoadmapSize"]  = 20 # number of nodes to add if there is no connection from start to end
            config["kNearest"] = 5 # number of nodes to connect to during setup
        """
        
        # HelperClass.HelperClass.printInColor("PLANNING ERROR ! PLANNING ERROR ! PLANNING ERROR", 'red')

        
        # 0. reset
        self.graph.clear()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedInterimGoalList, checkedGoalList = self._checkStartGoal(startList, interimGoalList, goalList)
        
        # Add Goallist to InterimGoalList
        checkedInterimGoalList.append(checkedGoalList[0])
        
        # 2. add start and goal to graph
        self.graph.add_node("start", pos=checkedStartList[0])
        # self.graph.add_node("interim", pos=checkedInterimGoalList[0])

        # Interim Liste erweitern
        for interimGoal in range(len(checkedInterimGoalList)):
            
            nameOfNode = "interim" + str(interimGoal)

            self.graph.add_node(nameOfNode, pos=checkedInterimGoalList[interimGoal])
            

        self.graph.add_node("goal", pos=checkedGoalList[0])
        
        # 3. build initial roadmap
        self._buildRoadmap(config["initialRoadmapSize"], config["kNearest"])
        
        maxTry = 0

        try:

            # Calculate shortest distance to nearest interim from start 
            result_interim = self._nearestInterim(checkedStartList[0], checkedInterimGoalList)
            print("Result:" + str(result_interim))
            
            # Plan path from start to nearest interim
            try_path = nx.shortest_path(self.graph, "start", result_interim[2])
            # print("PFAD: "+ str(try_path)+ str(type(try_path)))
            
            # Initialize path and loop break condition
            path = list()
            breakcondition = False
            
            # Loop to iteratively plan a path through interim goals
            while not breakcondition and maxTry < 40:
            #print(breakcondition)
                #try_path = try_path[:2]    
                print("TRYPATH :" + str(try_path))
                
                # Iterate through steps in the current try_path
                for step in try_path:
                    print("for-schleife beginnt")
                    
                    # Find nearest interim goal from the current step in Try-path
                    new_result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)
                    
                    # print("new_result_interim"+ str(new_result_interim))
                    
                    # Check if the new interim goal is the same as the previous one
                    if new_result_interim[2] == result_interim[2]:
                        print("New Result ist result interim ", new_result_interim[2])
                        # Add step to the final path
                        
                        # Check try_path for collision
                        if self._checkForCollisionAndUpdate(try_path):

                            print("Element 1", try_path[:1])
                            print("Element 2", try_path[:2])

                            # Remove actual step from path again
                            #a= path.pop()
                            #print(a)
                            
                            # Add nodes
                            self._buildRoadmap(config["updateRoadmapSize"], config["kNearest"])
                            
                            maxTry += 1
                            print("MaxTry: " + str(maxTry))
                            nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])
                            
                            # Plan a new path from the current position to the new interim goal
                            try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])

                            break
                        else:
                            path.append(step)
                        
                        HelperClass.HelperClass.printInColor("Aktueller Pfad: " + str(path), 'Dodgerblue')
                        print("Ziel-Interim" + str(result_interim))
                        print("Abstand" + str(new_result_interim[1]))
                        
                        # Check if the distance to the new interim is zero (Interim is reached)
                        if new_result_interim[1] == 0.0:
                            print("Interim ist erreicht")
                            
                            # Get the node name of current step based on coordinates
                            nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])
                            print("Aktuelle Interim Goallist: " +str(checkedInterimGoalList))
                            
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
                            
                            # Plan a new path from the current position to the new interim goal
                            try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])
                            
                            break

                    # If new interim goal is not the same as the current one  
                    else:
                        print("bin im Else")
                        
                        # Update the current interim goal through the new interim goal information
                        result_interim = new_result_interim
                        
                        # Get the node name of current step based on coordinates
                        nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])
                        
                        # Plan a new try path from the current position to the new interim goal
                        try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])

                        break
            
            
            
            
            HelperClass.HelperClass.printInColor(f"Pfad= {path}", 'green')
            print("")
            
            print("Vorher:", len(self.graph.nodes()))
            #self._checkForCollisionAndUpdate(self.graph.nodes())
            print("Nachher:", len(self.graph.nodes()))
            
            return path
        
        except Exception as e:
            HelperClass.HelperClass.printInColor("Kein Pfad gefunden", 'orange')
            print("MaxTry: "+ str(maxTry) + " Fehler: " + str(e))

        return []

    