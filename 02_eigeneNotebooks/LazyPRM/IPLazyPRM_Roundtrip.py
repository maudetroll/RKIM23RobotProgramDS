# coding: utf-8
"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from HelperPackage.IPPRMBase import PRMBase
from scipy.spatial import cKDTree
import networkx as nx
from scipy.spatial.distance import euclidean
from HelperPackage import HelperClass

from HelperPackage.IPPerfMonitor import IPPerfMonitor

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

        # first check all nodes
        if self._collisionChecker.pointInCollision(step):
            
            # Remove node if collision is detected
            self.graph.remove_node(self._getNodeNamebasedOnCoordinates(step))
            return True

        step = self._getNodeNamebasedOnCoordinates(step)
        path = self._getNodeNamebasedOnCoordinates(path) 

        # Check all edges
        if self._collisionChecker.lineInCollision(self.graph.nodes()[step]['pos'], self.graph.nodes()[path]['pos']):
            
            # Remove edge if collision is detected
            self.graph.remove_edge(step,path)
            self.collidingEdges.append((step,path))

            return True
        else:

            self.nonCollidingEdges.append((step,path))

        return False
    
    def _findDuplicate(self, subPath):
        
        # Initialize an empty list to store duplicate elements
        duplicate = []
        
        # Iterate over each element in the subPath list
        for i in range(len(subPath)):
            # Nested loop to compare each element with the remaining elements
            for j in range(i + 1, len(subPath)):
                
                # Check if the elements are equal and the element is not already in the 'duplicate' list
                if subPath[i] == subPath[j] and subPath[i] not in duplicate:
                    
                    # Add the duplicate element to the 'duplicate' list
                    duplicate.append(subPath[i])
        
        # If no duplicates were found, return an empty list
        if duplicate == []:
            return []
        
        # Return the first duplicate found (only the first one is returned, even if there are more)
        return duplicate[0]

    def _removeDuplicate(self, subPath, duplicate):
        
        # Find the indices of all occurrences of the duplicate element in the subPath list
        indexList = [index for index, element in enumerate(subPath) if element == duplicate]
        
        # Get the start and end indices of the duplicate occurrences
        start = indexList[0]
        end = indexList[-1]

        # Create a modified subPath by concatenating the portions before and after the duplicate occurrences
        modSubPath = subPath[:start + 1] + subPath[end + 1:]
        
        # Return the modified subPath without the duplicate occurrences
        return modSubPath
        
    @IPPerfMonitor   
    def planRoundPath(self, startList, interimGoalList, config):
        
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
        checkedStartList, checkedInterimGoalList = self._checkStartGoal(startList, interimGoalList)
        
        # 2. add start to graph
        self.graph.add_node("start", pos=checkedStartList[0])

        # Add Interims to graph
        for interimGoal in range(len(checkedInterimGoalList)):
            
            nameOfNode = "interim" + str(interimGoal)

            self.graph.add_node(nameOfNode, pos=checkedInterimGoalList[interimGoal])
        
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
            
            # Remove first element of try path
            try_path.remove('start')

            coordinatesLastPathEle.append(checkedStartList[0])
            
            # Initialize previous interim
            previousInterim = [[],[],'start']

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
                        
                        # Remove first path element to avoid duplicates in path
                        try_path.pop(0)

                        break
                        
                    else:
                        path.append(step)

                        # HelperClass.HelperClass.printInColor("Aktueller Pfad: " + str(path), 'Dodgerblue')
                        
                        # Save coordinates of last path element
                        coordinatesLastPathEle.append(self.graph.nodes()[step]['pos'])
                        
                        # Get actual nearest interim
                        new_result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)

                        # Check if interim is equal
                        if new_result_interim == result_interim:
                            
                            # Continue with next step of try path
                            continue                                    
                    
                        # Check if the distance to the new interim is zero (Interim is reached)
                        if new_result_interim[1] == 0.0:

                            # Optimize last path segment                          
                            subPath = path[path.index(previousInterim[2]):]

                            while(self._findDuplicate(subPath) != []):
                                duplicate = self._findDuplicate(subPath)
                                modSubPath = self._removeDuplicate(subPath, duplicate)
                                subPath = modSubPath
                            if subPath != path[path.index(previousInterim[2]):]:

                                # overwrite actual path with optimized path segment
                                path = path[:path.index(previousInterim[2])] + subPath


                            # Check if there is only one interim goal remaining, this means all interims are reached
                            if (len(checkedInterimGoalList) == 1 ):
                                
                                # End the loop
                                breakcondition = True
                                break
                            
                            # Save previous interim for case of optimization  
                            previousInterim = new_result_interim

                            # Remove the current interim goal from the list
                            checkedInterimGoalList.remove(new_result_interim[0])                          
                            
                            # Calculate the shortest distance to the new interim goal
                            result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)
                            
                            nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes()[step]['pos'])

                            # Plan a new path from the current position to the new interim goal
                            try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])
                            
                            try_path.pop(0)

                            break
                        
                        # Check if nearest interim has changed
                        if new_result_interim != result_interim:
                            
                            # Save old interim for case of looping
                            old_resultInterim = result_interim

                            # Change interim goal to new closest interim goal
                            result_interim = new_result_interim

                            # Plan a new path from the current position to the new interim goal
                            nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes()[step]['pos'])
                            try_path = nx.shortest_path(self.graph,nodeName,result_interim[2])
                            try_path.pop(0)

                            # Avoid looping by detecting looping pattern
                            
                            # Check if the length of the 'path' list is greater than 2
                            if len(path) > 2:
                                
                                # Check if the last element of 'path' is equal to third last element of 'path'
                                # and the first element of 'try_path' is equal to the second last element of 'path'
                                if path[-1] == path[-3] and try_path[0] == path[-2]:
                                    
                                    # Break looping by planning path to old_resultInterim 
                                    try_path = nx.shortest_path(self.graph,nodeName,old_resultInterim[2])
                                    try_path.pop(0)
                                    
                                    # Update the actual interim goal
                                    result_interim = old_resultInterim

                            break

            if maxTry == maxIterations:
                # Empty the actual path if maxIterations is reached
                path = []
                HelperClass.HelperClass.printInColor("No Path found", 'red')
            else:  
                HelperClass.HelperClass.printInColor("Solution =  " + str(path), 'lawngreen')

            return path
        
        except Exception as e:
            HelperClass.HelperClass.printInColor("No Path found", 'red')

        return []

    