# coding: utf-8

"""
This code is part of the course 'Introduction to robot path planning' (Author: Bjoern Hein). 

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

class Benchmark(object):
        
    def __init__(self, name, collisionChecker, startList,interimGoalList, description, level ):
        """Benchmark is used to define tests cases
        
        :name: Name of benchmark
        :environment: collision checker
        :startList: list of possible start configurations
        :goalList: list of possible goal configurations
        :description: textual description of the benchmark
        :level (int): 1 -"easy",2 - "normal",3 - "hard", 4 - "insane"
        """
        self.name = name
        self.collisionChecker = collisionChecker # environment (collision checker) 
        self.startList = startList # in limits, collision-free?
        self.interimGoalList = interimGoalList
        self.description = description
        self.level = level # in [1,2,3,4]