# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPRMBase import PRMBase
import networkx as nx
from scipy.spatial import cKDTree
from IPPerfMonitor import IPPerfMonitor

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

        for interimGoal in range(len(checkedInterimGoalList)):
            print("InterimGoal: " + str(interimGoal))
            print("Was steht in der Liste: " + str(checkedInterimGoalList[interimGoal]))
            result = kdTree.query(checkedInterimGoalList[interimGoal],k=5)
            
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
        except:
            return []
        return path
        
