# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from HelperPackage.IPBenchmark import Benchmark 
from HelperPackage.IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
import shapely.affinity
import math
import numpy as np
import random

benchList = list()

# -----------------------------------------
trapField = dict()
trapField["obs1"] =   LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(1.0)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionChecker(trapField), [[9,15]], [[3,9],[19,8],[3,3],[20,15],[10,1]], description, 2))


# -----------------------------------------
bottleNeckField = dict()
bottleNeckField["obs1"] = LineString([(0, 13), (11, 13)]).buffer(.5)
bottleNeckField["obs2"] = LineString([(13, 13), (23,13)]).buffer(.5)
description = "Planer has to find a narrow passage."
benchList.append(Benchmark("Bottleneck", CollisionChecker(bottleNeckField), [[4,15]],[[18,3],[18,16],[4,4],[18,1]], description, 2))



# -----------------------------------------
fatBottleNeckField = dict()
fatBottleNeckField["obs1"] = Polygon([(0, 8), (11, 8),(11, 15), (0, 15)]).buffer(.5)
fatBottleNeckField["obs2"] = Polygon([(13, 8), (24, 8),(24, 15), (13, 15)]).buffer(.5)
description = "Planer has to find a narrow passage with a significant extend."
benchList.append(Benchmark("Fat bottleneck", CollisionChecker(fatBottleNeckField), [[4,21]], [[18,4],[4,4],[18,16],[18,1]], description, 2))

# BE RUSH

simpleField = dict()
simpleField["obs1"] = Polygon([(0,5),(5,5),(5,4),(0,4)])
simpleField["obs2"] = Polygon([(10,5),(18,5),(18,4),(10,4)])
simpleField["obs3"] = Polygon([(21,5),(23,5),(23,4),(21,4)])
simpleField["obs4"] = Polygon([(10,23),(10,20),(11,20),(11,23)])
simpleField["obs5"] = Polygon([(10,16),(10,13),(11,13),(11,16)])
simpleField["obs6"] = Polygon([(11,13),(23,13),(23,14),(11,14)])
simpleField["obs7"] = Polygon([(0,21),(2,21),(2,23),(0,23)])
simpleField["obs8"] = Polygon([(0,17),(3.5,17),(3.5,15),(0,15)])
simpleField["obs9"] = Polygon([(0.5,7),(2.5,5.5),(3.5,6.5),(1.5,8)])
simpleField["obs10"] = Polygon([(0,10),(2,10),(2,12.5),(1,12.5),(1,11.25),(0,11.25)])
simpleField["obs11"] = Polygon([(6,23),(6,21.5),(7,21.5),(7,23)])
simpleField["obs12"] = Polygon([(18,23),(18,20),(20,20),(20,23)])
simpleField["obs13"] = Polygon([(11,14),(13,14),(13,17),(11,17)])
simpleField["obs14"] = Point(18,15.5).buffer(0.9)
simpleField["obs15"] = Polygon([(5,4),(8,3),(8,3.5),(5,4.5)])
simpleField["obs16"] = Polygon([(10,5),(7,6),(7,5.5),(10,4.5)])
simpleField["obs17"] = Polygon([(13.5,5),(16.5,5),(16.5,7.5),(13.5,7.5)])
simpleField["obs18"] = Polygon([(13+0.5,5+4),(17-0.5,5+4),(17-0.5,8+4),(13+0.5,8+4)])
simpleField["obs19"] = Polygon([(18,7),(22,7),(22,10),(18,10)])
simpleField["obs20"] = Polygon([(0,0.5),(7,0.5),(7,2),(0,2)])

description = "Find the path!"
benchList.append(Benchmark("B_rush",CollisionChecker(simpleField),[[1,17.7]],[[10,10],[15,2],[20,18],[17.25,10]], description,2))


### Circle of Death

calc = dict()
draw = dict()
mid = list([10,10])
circle_size = 1
circle_dis = 3
circle_anz = 3
cut_with = 1

for i in range(circle_anz):
    calc["A"+str(i)] = Point(mid).buffer(2*circle_size+circle_dis*i)
    calc["B"+str(i)] = Point(mid).buffer(1*circle_size+circle_dis*i)
    a = random.randint(0,360)
    P_a = list([math.cos(a)*(circle_dis*circle_anz)+mid[0],math.sin(a)*(circle_dis*circle_anz)+mid[1]])
    calc["C"+str(i)] = LineString([(P_a),mid]).buffer(cut_with)
    calc["dif"+str(i)] = (calc["A"+str(i)].difference(calc["B"+str(i)])).difference(calc["C"+str(i)])
    draw["sol_1"+str(i)] = calc["dif"+str(i)]

description = "Find the path"
benchList.append(Benchmark("Circle of Death", CollisionChecker(draw), [mid],[[10,4.5],[10,7.5],[20,3],[1,1]], description, 2))