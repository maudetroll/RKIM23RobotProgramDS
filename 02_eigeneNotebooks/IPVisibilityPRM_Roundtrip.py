# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPRMBase import PRMBase
import networkx as nx
from scipy.spatial import cKDTree
from IPPerfMonitor import IPPerfMonitor
from scipy.spatial.distance import euclidean

class VisibilityStatsHandler():
    
    def __init__(self):
        self.graph = nx.Graph()
        
    def addNodeAtPos(self,nodeNumber,pos):
        self.graph.add_node(nodeNumber, pos=pos, color='yellow')
        return
    
    def addVisTest(self,fr,to):
        self.graph.add_edge(fr, to)
        return
        
class VisPRM(PRMBase):
    """Class implements an simplified version of a visibility PRM"""

    def __init__(self, _collChecker, _statsHandler = None):
        super(VisPRM, self).__init__(_collChecker)
        self.graph = nx.Graph()
        self.statsHandler = VisibilityStatsHandler() # not yet fully customizable (s. parameters of constructors)
                
    def _isVisible(self, pos, guardPos):
        return not self._collisionChecker.lineInCollision(pos, guardPos)

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
    def _learnRoadmap(self, ntry):

        nodeNumber = 0
        currTry = 0
        while currTry < ntry:
            #print currTry
            # select a random  free position
            q_pos = self._getRandomFreePosition()
            if self.statsHandler:
                self.statsHandler.addNodeAtPos(nodeNumber, q_pos)
           
            g_vis = None
        
            # every connected component represents one guard
            merged = False
            for comp in nx.connected_components(self.graph): # Impliciteley represents G_vis
                found = False
                merged = False
                for g in comp: # connected components consists of guards and connection: only test nodes of type 'Guards'
                    if self.graph.nodes()[g]['nodeType'] == 'Guard':
                        if self.statsHandler:
                            self.statsHandler.addVisTest(nodeNumber, g)
                        if self._isVisible(q_pos,self.graph.nodes()[g]['pos']):
                            found = True
                            if g_vis == None:
                                g_vis = g
                            else:
                                self.graph.add_node(nodeNumber, pos = q_pos, color='lightblue', nodeType = 'Connection')
                                self.graph.add_edge(nodeNumber, g)
                                self.graph.add_edge(nodeNumber, g_vis)
                                #print "ADDED Connection node", nodeNumber
                                merged = True
                        # break, if node was visible,because visibility from one node of the guard is sufficient...
                        if found == True: break;
                # break, if connection was found. Reason: computed connected components (comp) are not correct any more, 
                # they've changed because of merging
                if merged == True: # how  does it change the behaviour? What has to be done to keep the original behaviour?
                    break;                    

            if (merged==False) and (g_vis == None):
                self.graph.add_node(nodeNumber, pos = q_pos, color='red', nodeType = 'Guard')
                #print "ADDED Guard ", nodeNumber
                currTry = 0
            else:
                currTry += 1

            nodeNumber += 1

    @IPPerfMonitor
    def planRoundPath(self, startList, interimGoalList, goalList, config):
        """
        
        Args:
            start (array): start position in planning space
            goal (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options
            
        Example:
        
            config["ntry"] = 40 
        
        """
        # 0. reset
        self.graph.clear()
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedInterimGoalList, checkedGoalList = self._checkStartGoal(startList, interimGoalList, goalList)
        
        # Add Goallist to InterimGoalList
        checkedInterimGoalList.append(checkedGoalList[0])

        # 2. learn Roadmap
        self._learnRoadmap(config["ntry"])

        # 3. find connection of start and goal to roadmap
        # find nearest, collision-free connection between node on graph and start
        posList = nx.get_node_attributes(self.graph,'pos')
        kdTree = cKDTree(list(posList.values()))
        
        result = kdTree.query(checkedStartList[0],k=5)
        for node in result[1]:
            if not self._collisionChecker.lineInCollision(checkedStartList[0],self.graph.nodes()[list(posList.keys())[node]]['pos']):
                 self.graph.add_node("start", pos=checkedStartList[0], color='lightgreen')
                 self.graph.add_edge("start", list(posList.keys())[node])
                 break
        
        # Iterate through each interim goal in the list
        for interimGoal in range(len(checkedInterimGoalList)):
            print("InterimGoal: " + str(interimGoal))
            # print("Was steht in der Liste: " + str(checkedInterimGoalList[interimGoal]))
            result = kdTree.query(checkedInterimGoalList[interimGoal],k=5)
            
            # Create a unique name for the current interim goal node
            nameOfNode = "interim" + str(interimGoal)

            for node in result[1]:
                if not self._collisionChecker.lineInCollision(checkedInterimGoalList[interimGoal],self.graph.nodes()[list(posList.keys())[node]]['pos']):
                     self.graph.add_node(nameOfNode, pos=checkedInterimGoalList[interimGoal], color='lightgreen')
                     self.graph.add_edge(nameOfNode, list(posList.keys())[node])
                     break
                    
                    
        result = kdTree.query(checkedGoalList[0],k=5)
        for node in result[1]:
            if not self._collisionChecker.lineInCollision(checkedGoalList[0],self.graph.nodes()[list(posList.keys())[node]]['pos']):
                 self.graph.add_node("goal", pos=checkedGoalList[0], color='lightgreen')
                 self.graph.add_edge("goal", list(posList.keys())[node])
                 break

                    
        print(self.graph.nodes())        
        try:
            # Calculate shortest distance to nearest interim from start 
            result_interim = self._nearestInterim(checkedStartList[0], checkedInterimGoalList)
            print("Result:" + str(result_interim))
            
            # Plan path from start to nearest interim
            try_path = nx.shortest_path(self.graph, "start", result_interim[2])
            print("PFAD: "+ str(try_path)+ str(type(try_path)))
            
            # Initialize path and loop break condition
            path = list()
            breakcondition = False
            
            # Loop to iteratively plan a path through interim goals
            while not breakcondition:
            #print(breakcondition)
                    
                print("TRYPATH :" + str(try_path))
                
                # Iterate through steps in the current try_path
                for step in try_path:
                    print("for-schleife beginnt")
                    
                    # Find nearest interim goal from the current step in Try-path
                    new_result_interim = self._nearestInterim(self.graph.nodes[step]['pos'], checkedInterimGoalList)
                    
                    # print("new_result_interim"+ str(new_result_interim))
                    
                    # Check if the new interim goal is the same as the previous one
                    if new_result_interim[2] == result_interim[2]:
                        
                        # Add step to the final path
                        path.append(step)
                        
                        print("aktuelles Interim" + str(result_interim))
                        print("Abstand" + str(new_result_interim[1]))
                        
                        # Check if the distance to the new interim is zero (Interim is reached)
                        if new_result_interim[1] == 0.0:
                            print("DER ABSTAND IST NULL")
                            
                            # Get the node name of current step based on coordinates
                            nodeName = self._getNodeNamebasedOnCoordinates(self.graph.nodes[step]['pos'])
                            print("Akutlele Interim Golalist: " +str(checkedInterimGoalList))
                            
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
               
            print("Kompletter Pfad: "+ str(path))


            
            """
            interim_count = len(checkedInterimGoalList)

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
            """
        except Exception as e :
            print("Fehler " + str(e))
            return []
        return path
        
