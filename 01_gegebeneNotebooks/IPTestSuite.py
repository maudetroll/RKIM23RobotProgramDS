# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPBenchmark import Benchmark 
from IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
import shapely.affinity
import math
import numpy as np


benchList = list()

# -----------------------------------------
# trapField = dict()
# trapField["obs1"] =   LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(1.0)
# description = "Following the direct connection from goal to start would lead the algorithm into a trap."
# benchList.append(Benchmark("Trap", CollisionChecker(trapField), [[10,15]], [[10,1]], description, 2))

# -----------------------------------------
trapField = dict()
trapField["obs1"] =   LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(1.0)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionChecker(trapField), [[10,15]], [[20,10]], [[10,1]], description, 2))

# # -----------------------------------------
# bottleNeckField = dict()
# bottleNeckField["obs1"] = LineString([(0, 13), (11, 13)]).buffer(.5)
# bottleNeckField["obs2"] = LineString([(13, 13), (23,13)]).buffer(.5)
# description = "Planer has to find a narrow passage."
# benchList.append(Benchmark("Bottleneck", CollisionChecker(bottleNeckField), [[4,15]], [[18,1]], description, 2))

# -----------------------------------------
bottleNeckField = dict()
bottleNeckField["obs1"] = LineString([(0, 13), (11, 13)]).buffer(.5)
bottleNeckField["obs2"] = LineString([(13, 13), (23,13)]).buffer(.5)
description = "Planer has to find a narrow passage."
benchList.append(Benchmark("Bottleneck", CollisionChecker(bottleNeckField), [[4,15]],[[18,3]], [[18,1]], description, 2))

# # -----------------------------------------
# fatBottleNeckField = dict()
# fatBottleNeckField["obs1"] = Polygon([(0, 8), (11, 8),(11, 15), (0, 15)]).buffer(.5)
# fatBottleNeckField["obs2"] = Polygon([(13, 8), (24, 8),(24, 15), (13, 15)]).buffer(.5)
# description = "Planer has to find a narrow passage with a significant extend."
# benchList.append(Benchmark("Fat bottleneck", CollisionChecker(fatBottleNeckField), [[4,21]], [[18,1]], description, 2))

# -----------------------------------------
fatBottleNeckField = dict()
fatBottleNeckField["obs1"] = Polygon([(0, 8), (11, 8),(11, 15), (0, 15)]).buffer(.5)
fatBottleNeckField["obs2"] = Polygon([(13, 8), (24, 8),(24, 15), (13, 15)]).buffer(.5)
description = "Planer has to find a narrow passage with a significant extend."
benchList.append(Benchmark("Fat bottleneck", CollisionChecker(fatBottleNeckField), [[4,21]], [[18,3]], [[18,1]], description, 2))

# # -----------------------------------------
# trapField = dict()
# for i in range(10, 1300, 10) :
#     radius = 1.0 * (i / 500.0)
#     width = 1.0 * (i / 5000.0)
#     trapField["obsA"+str(i/10)] = Point([(10 - np.cos(np.deg2rad(i))*radius, 10 - np.sin(np.deg2rad(i))*radius)]).buffer(width)
#     trapField["obsB"+str(i/10)] = Point([(15 + np.sin(np.deg2rad(i))*radius, 15 + np.cos(np.deg2rad(i))*radius)]).buffer(width)
# trapField["obsC"] = LineString([(5, 0.5), (5, 10), (15, 20), (20,20)]).buffer(0.5)

# start = [[10,10]]
# interim = [[10, 9]]
# goal = [[15,15]]

# description = "Two spirals block the way from start to goal."
# benchList.append(Benchmark("Spirals", CollisionChecker(trapField), start,interim, goal, description, 4))

