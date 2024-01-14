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
    def _checkForCollisionAndUpdate(self,step, path):
        #path = long_path[:2]

        # first check all nodes
        if self._collisionChecker.pointInCollision(step):

            self.graph.remove_node(self._getNodeNamebasedOnCoordinates(step))
            return True
        
        # check all path segments

        step = self._getNodeNamebasedOnCoordinates(step)
        path = self._getNodeNamebasedOnCoordinates(path) 
    
        if self._collisionChecker.lineInCollision(self.graph.nodes()[step]['pos'], self.graph.nodes()[path]['pos']):
            self.graph.remove_edge(step,path)
            self.collidingEdges.append((step,path))

            return True
        else:

            self.nonCollidingEdges.append((step,path))

        return False
    
    def _findDuplicate(self, subPath):

        
        duplicate = []
        
        for i in range(len(subPath)):
            for j in range(i + 1, len(subPath)):
                
                if subPath[i] == subPath[j] and subPath[i] not in duplicate:
                    duplicate.append(subPath[i])
        
        if duplicate == []:
            return []
        
        return duplicate[0]

    def _removeDuplicate(self, subPath, duplicate):
        
        # find index of duplicate
        indexList = [index for index, element in enumerate(subPath) if element == duplicate]
        start = indexList[0]
        end = indexList[-1]
        modSubPath = subPath[:start + 1] + subPath[end + 1:]
        
        return modSubPath
        
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
        coordinatesLastPathEle = []
        maxIterations = 100

        try:

            # Calculate shortest distance to nearest interim from start 
            result_interim = self._nearestInterim(checkedStartList[0], checkedInterimGoalList)

            # Initialize path and loop break condition
            path = list()
            breakcondition = False
            
            # Plan path from start to nearest interim
            try_path = nx.shortest_path(self.graph, "start", result_interim[2])
            path.append(try_path[0])
            
            # Löschen des ersten Elements -> Start
            try_path.remove('start')

            coordinatesLastPathEle.append(checkedStartList[0])
            oldInterim = [[],[],'start']
            # Loop to iteratively plan a path through interim goals
            while not breakcondition and maxTry < maxIterations:
     
                # Iterate through steps in the current try_path
                for step in try_path:

                    # Check for collision between current step and last path element
                    if self._checkForCollisionAndUpdate(self.graph.nodes[step]['pos'], coordinatesLastPathEle[-1]):
                        
                        # Add nodes
                        self._buildRoadmap(config["updateRoadmapSize"], config["kNearest"])
                        
                        maxTry += 1                        
                        
                        # Plan a new path from the current position to the new interim goal
                        try_path = nx.shortest_path(self.graph,path[-1],result_interim[2])
                        
                        try_path.pop(0)

                        break
                        
                    else:
                        path.append(step)

                        HelperClass.HelperClass.printInColor("Aktueller Pfad: " + str(path), 'Dodgerblue')
                        
                        coordinatesLastPathEle.append(self.graph.nodes()[step]['pos'])
                        new_result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)


                        if new_result_interim == result_interim:

                            continue                                    
                    
                        # Check if the distance to the new interim is zero (Interim is reached)
                        if new_result_interim[1] == 0.0:

                                                       
                            subPath = path[path.index(oldInterim[2]):]


                            while(self._findDuplicate(subPath) != []):
                                duplicate = self._findDuplicate(subPath)
                                modSubPath = self._removeDuplicate(subPath, duplicate)
                                subPath = modSubPath

                            if subPath != path[path.index(oldInterim[2]):]:
                                path = path[:path.index(oldInterim[2])] + subPath

                            # Check if there is only one interim goal remaining, this means all interims are reached
                            if (len(checkedInterimGoalList) == 1 ):
                                
                                # End the loop
                                breakcondition = True
                                break
                                
                            # Remove the current interim goal from the list

                            oldInterim = new_result_interim
                            checkedInterimGoalList.remove(new_result_interim[0])                          
                            
                            # Calculate the shortest distance to the new interim goal
                            result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)

                            nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes()[step]['pos'])
                            #nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes()[path[-1]]['pos'])

                            # Plan a new path from the current position to the new interim goal
                            try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])
                            
                            try_path.pop(0)

                            break
            
            if maxTry == maxIterations:
                path = []
            HelperClass.HelperClass.printInColor("Solution =  " + str(path), 'lawngreen')

            return path
        
        except Exception as e:
            HelperClass.HelperClass.printInColor("Kein Pfad gefunden", 'orange')
            print("MaxTry: "+ str(maxTry) + " Fehler: " + str(e))
            import traceback
            traceback.print_exc()

        return []

    