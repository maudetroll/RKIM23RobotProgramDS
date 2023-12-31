# coding: utf-8
"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPRMBase import PRMBase
from scipy.spatial import cKDTree
import networkx as nx
import random

from IPPerfMonitor import IPPerfMonitor

class LazyPRM(PRMBase):

    def __init__(self, _collChecker):
        super(LazyPRM, self).__init__(_collChecker)
        
        self.graph = nx.Graph()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []
        self.nonCollidingEdges =[]
        
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
        #print posList
        kdTree = cKDTree(posList)
        
        # to see when _buildRoadmap has to be called again
        #print addedNodes
        for node in addedNodes:
        #for node in self.graph.nodes():
        # Find set of candidates to connect to sorted by distance
            result = kdTree.query(self.graph.nodes[node]['pos'],k=kNearest)
            for data in result[1]:
                c_node = [x for x, y in self.graph.nodes(data=True) if (y['pos']==posList[data])][0]
                if node!=c_node:
                    if (node, c_node) not in self.collidingEdges:
                        self.graph.add_edge(node,c_node)
                    else:
                        continue
                        #print "not adding already checked colliding edge"
    
    @IPPerfMonitor
    def _checkForCollisionAndUpdate(self,path):
        # first check all nodes
        for nodeNumber in path:
            if self._collisionChecker.pointInCollision(self.graph.nodes[nodeNumber]['pos']):
                self.graph.remove_node(nodeNumber)
                
                #print "Colliding Node"
                return True
        
        # check all path segments
            
        x_old = ()
        y_old = ()    
        for elem in zip(path,path[1:]):
            #print elem
            x = elem[0]
            y = elem[1]
            print("Elem: "+ str(elem))
            # if x == y:
            #     self.graph.remove_edge(x,y)
            #     self.collidingEdges.append((x,y))
            #     if y_old == ():
            #         self.graph.add_edge("start", x)
            #     else:
            #         self.graph.add_edge(y_old, x)

            #     print("Kanten entfernt")

            if self._collisionChecker.lineInCollision(self.graph.nodes()[x]['pos'],self.graph.nodes()[y]['pos']):
                self.graph.remove_edge(x,y)
                self.collidingEdges.append((x,y))
                return True
            else:
                self.nonCollidingEdges.append((x,y))
            
            x_old = x
            y_old = y

        print("Colliding Edges= " + str(self.collidingEdges))
                                                                                          
            
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
        # 0. reset
        self.graph.clear()
        self.lastGeneratedNodeNumber = 0
        self.collidingEdges = []
        
        # 1. check start and goal whether collision free (s. BaseClass)
        checkedStartList, checkedInterimGoalList, checkedGoalList = self._checkStartGoal(startList, interimGoalList, goalList)
        
        # 2. add start and goal to graph
        self.graph.add_node("start", pos=checkedStartList[0])
        # self.graph.add_node("interim", pos=checkedInterimGoalList[0])

        # Interim Liste erweitern
        for interimGoal in range(len(checkedInterimGoalList)):
            print("InterimGoal: " + str(interimGoal))
            print("Was steht in der Liste: " + str(checkedInterimGoalList[interimGoal]))
            
            nameOfNode = "interim" + str(interimGoal)

            self.graph.add_node(nameOfNode, pos=checkedInterimGoalList[interimGoal])
            

        self.graph.add_node("goal", pos=checkedGoalList[0])
        
        # 3. build initial roadmap
        self._buildRoadmap(config["initialRoadmapSize"], config["kNearest"])
        
        maxTry = 0
        while maxTry < 40: 
            try:
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
                  
            except:
                self._buildRoadmap(config["updateRoadmapSize"], config["kNearest"])
                maxTry += 1
                continue
  
            if self._checkForCollisionAndUpdate(path):
                continue
            else:
                #print "Found solution"
                print(f"Pfad= {path}")
                return path
               
        return []

    